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
    const container = getContainer(env.COIL_CONTAINER, "singleton");
    return container.fetch(request);
  },
};
