const loading = document.querySelector("#loading");
const startupIcon = document.querySelector("#startup-icon");
const startup = {
  show(icon, title, detail) {
    startupIcon.style.setProperty("--startup-icon", `url("icons/${icon}.svg")`);
    document.querySelector("#startup-title").textContent = title;
    document.querySelector("#startup-detail").textContent = detail;
  },
  transfer(text) {
    document.querySelector("#startup-transfer").textContent = text;
  },
  background() {
    loading.classList.add("background");
  },
  finish(delay = 3500) {
    loading.classList.add("done");
    setTimeout(() => loading.remove(), delay);
  },
};
window.nanoplanStartup = startup;

let appReady = false;
let catalogPhase = "pending";
let downloadedTracks = 0;
const showCatalogPhase = () => {
  if (!appReady) return;
  startup.background();
  if (catalogPhase === "cache") {
    startup.show(
      "refresh-cw",
      "Driving app ready",
      "Checking the browser cache for the circuits.",
    );
  } else if (catalogPhase === "download") {
    startup.show(
      "download",
      "Driving app ready · receiving circuits",
      downloadedTracks +
        " circuits downloaded in parallel. You can start driving now.",
    );
  } else if (catalogPhase === "store") {
    startup.show(
      "save",
      "Driving app ready · saving circuits",
      "Storing the downloaded circuit catalog. Each circuit is prepared only when selected.",
    );
  } else if (catalogPhase === "ready") {
    startup.show(
      "circle-check",
      "Nanoplan fully loaded",
      "The renderer and motion planners are ready, and all circuits are available.",
    );
    startup.finish();
  } else if (catalogPhase === "failed") {
    startup.show(
      "triangle-alert",
      "Nanoplan ready with built-in circuits",
      "The circuit catalog could not be loaded; generated and preset circuits remain available.",
    );
    startup.finish(8000);
  } else {
    startup.show(
      "flag",
      "Driving app ready",
      "You can start now. The optional circuit catalog is finishing in the background.",
    );
  }
};

new PerformanceObserver((list) => {
  for (const entry of list.getEntries()) {
    if (entry.name === "nanoplan-track-catalog-cache-check") catalogPhase = "cache";
    if (entry.name === "nanoplan-track-catalog-download-start") catalogPhase = "download";
    if (entry.name === "nanoplan-track-downloaded") {
      catalogPhase = "download";
      downloadedTracks += 1;
    }
    if (entry.name === "nanoplan-track-catalog-store") catalogPhase = "store";
    if (entry.name === "nanoplan-track-catalog-ready") catalogPhase = "ready";
    if (entry.name === "nanoplan-track-catalog-failed") catalogPhase = "failed";
  }
  showCatalogPhase();
}).observe({ type: "mark", buffered: true });

addEventListener(
  "TrunkApplicationStarted",
  () => {
    const wasmReady = performance.getEntriesByName("nanoplan-wasm-ready")[0];
    if (!wasmReady) return;
    requestAnimationFrame(() =>
      requestAnimationFrame(() => {
        performance.mark("nanoplan-app-ready");
        appReady = true;
        showCatalogPhase();
        const firstPaint = performance.getEntriesByName("first-contentful-paint")[0];
        console.info("nanoplan startup", {
          firstPaintMs: firstPaint?.startTime,
          wasmReadyMs: wasmReady.startTime,
          appReadyMs: performance.getEntriesByName("nanoplan-app-ready")[0].startTime,
        });
      }),
    );
  },
  { once: true },
);
