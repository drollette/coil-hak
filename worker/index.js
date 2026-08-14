import { Container, getContainer } from "@cloudflare/containers";

// Shared coil-design links. Handled entirely here in the Worker (not
// proxied to the container) since the KV binding lives on the Worker/
// Durable Object side, not inside the Docker container — the FastAPI
// backend never sees these requests or needs to know about KV at all.
//
// Storage: SHARES_KV is the same underlying namespace as w7hak.com's
// guestbook (see wrangler.jsonc), so keys are prefixed to keep the two
// datasets apart. IDs are 8 lowercase-hex characters, which conveniently
// can never collide with any real FastAPI route name below (they're all
// non-hex letters and/or the wrong length), so no separate path-exclusion
// list is needed for the rewrite in the default fetch handler.
const SHARE_KEY_PREFIX = "coilshare:";
const SHARE_ID_PATTERN = /^[0-9a-f]{8}$/;
const SHARE_MAX_BODY_BYTES = 8192; // generous; real param sets are <1KB

function jsonResponse(body, status = 200) {
  return new Response(JSON.stringify(body), {
    status,
    headers: { "Content-Type": "application/json" },
  });
}

async function generateUniqueShareId(kv) {
  for (let attempt = 0; attempt < 5; attempt++) {
    const id = crypto.randomUUID().replace(/-/g, "").slice(0, 8);
    const existing = await kv.get(SHARE_KEY_PREFIX + id);
    if (existing === null) return id;
  }
  throw new Error("Could not allocate a unique share ID");
}

async function handleCreateShare(request, env) {
  const contentLength = Number(request.headers.get("content-length") || 0);
  if (contentLength > SHARE_MAX_BODY_BYTES) {
    return jsonResponse({ error: "Request body too large" }, 413);
  }

  const rawBody = await request.text();
  if (rawBody.length > SHARE_MAX_BODY_BYTES) {
    return jsonResponse({ error: "Request body too large" }, 413);
  }

  let params;
  try {
    params = JSON.parse(rawBody);
  } catch (err) {
    return jsonResponse({ error: "Invalid JSON body" }, 400);
  }
  if (typeof params !== "object" || params === null || Array.isArray(params)) {
    return jsonResponse({ error: "Body must be a JSON object" }, 400);
  }

  const id = await generateUniqueShareId(env.SHARES_KV);
  await env.SHARES_KV.put(
    SHARE_KEY_PREFIX + id,
    JSON.stringify({ params, created_at: Date.now() })
  );
  return jsonResponse({ id }, 201);
}

async function handleGetShare(env, id) {
  if (!SHARE_ID_PATTERN.test(id)) {
    return jsonResponse({ error: "Not found" }, 404);
  }
  const stored = await env.SHARES_KV.get(SHARE_KEY_PREFIX + id);
  if (stored === null) {
    return jsonResponse({ error: "Not found" }, 404);
  }
  return new Response(stored, {
    status: 200,
    headers: { "Content-Type": "application/json" },
  });
}

// Returns a Response if this request is a share-API call, or null if the
// caller should fall through to the normal container proxy.
async function handleShareRoutes(request, env, url) {
  if (url.pathname === "/coil/api/share") {
    if (request.method === "POST") return handleCreateShare(request, env);
    return jsonResponse({ error: "Method not allowed" }, 405);
  }
  const shareMatch = url.pathname.match(/^\/coil\/api\/share\/([^/]+)$/);
  if (shareMatch) {
    if (request.method === "GET") return handleGetShare(env, shareMatch[1]);
    return jsonResponse({ error: "Method not allowed" }, 405);
  }
  return null;
}

// Cloudflare Container wrapping the existing FastAPI/CadQuery Dockerfile.
// One instance for everything ("singleton"): the backend writes generated
// STL/STEP files to local disk keyed by job UUID, so a later download must
// hit the same instance that ran the /generate request.
export class CoilFormerContainer extends Container {
  defaultPort = 8000;
  sleepAfter = "10m";
  envVars = {
    // Tells the FastAPI app it's mounted at /coil rather than the domain
    // root, so it serves routes/static files and builds download URLs
    // under that prefix. See backend/main.py.
    ROUTE_PREFIX: "/coil",
  };

  // The default port-readiness wait (20s) is too short for this image: it's
  // a mamba/conda environment that has to import CadQuery/OpenCASCADE before
  // uvicorn even starts listening, which routinely takes longer than that on
  // a cold start (observed up to ~100s). Without this override, requests
  // that arrive while the container is still starting fail outright with
  // "container is not listening" / "container is not running" instead of
  // waiting for it. See https://github.com/cloudflare/containers/issues/139.
  async fetch(request) {
    await this.startAndWaitForPorts({
      ports: [this.defaultPort],
      cancellationOptions: {
        portReadyTimeoutMS: 120_000,
      },
    });
    return this.containerFetch(request);
  }
}

export default {
  async fetch(request, env) {
    const url = new URL(request.url);

    const shareResponse = await handleShareRoutes(request, env, url);
    if (shareResponse) return shareResponse;

    // A bare /coil/{8-hex-chars} path is a shared-design link. The
    // container has no file at that path (its static mount only knows
    // about index.html — see backend/main.py), so rewrite the outbound
    // request to serve the SPA shell instead of a 404. This only changes
    // what's sent to the container; the browser's address bar keeps
    // showing /coil/{id}, which the frontend reads client-side (see
    // wasm-coil-former/static/index.html) to fetch the design from
    // /coil/api/share/{id} and load it in.
    const bareSegment = url.pathname.replace(/^\/coil\/?/, "");
    if (SHARE_ID_PATTERN.test(bareSegment)) {
      const rewritten = new URL(url);
      rewritten.pathname = "/coil/";
      request = new Request(rewritten, request);
    }

    const container = getContainer(env.COIL_CONTAINER, "singleton");
    return container.fetch(request);
  },
};
