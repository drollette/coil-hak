import { Container, getContainer } from "@cloudflare/containers";

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
}

export default {
  async fetch(request, env) {
    const container = getContainer(env.COIL_CONTAINER, "singleton");
    return container.fetch(request);
  },
};
