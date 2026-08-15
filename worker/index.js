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

// The Cloudflare Container (Python/CadQuery FastAPI backend) this Worker
// used to proxy to is retired -- deleted, along with the wrangler.jsonc
// "routes" entry that claimed w7hak.com/coil/* on the zone, in favor of
// a client-side (replicad/OpenCascade.js) port living in the w7hak.com
// repo (src/lib/coilGeometry.js), served natively by that Pages project.
// With no route bound, nothing on the zone reaches this Worker's fetch
// handler anymore -- it's left deployed but unreachable rather than
// deleted outright, since handleShareRoutes()'s share-link create/load
// logic (still fully functional, independent of the container) may be
// useful reference for rebuilding sharing natively via w7hak.com Pages
// Functions (see that repo's task list).
export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    const shareResponse = await handleShareRoutes(request, env, url);
    if (shareResponse) return shareResponse;
    return new Response("Not found", { status: 404 });
  },
};
