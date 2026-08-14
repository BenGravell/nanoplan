const loading = document.querySelector("#loading");
const startupIcon = document.querySelector("#startup-icon");
document.querySelector('meta[name="theme-color"]').content = getComputedStyle(document.documentElement)
  .getPropertyValue("--orange")
  .trim();
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

addEventListener(
  "TrunkApplicationStarted",
  () => {
    const wasmReady = performance.getEntriesByName("nanoplan-wasm-ready")[0];
    if (!wasmReady) return;
    requestAnimationFrame(() =>
      requestAnimationFrame(() => {
        performance.mark("nanoplan-app-ready");
        startup.background();
        startup.show(
          "circle-check",
          "Nanoplan fully loaded",
          "The renderer, motion planners, and circuits are ready.",
        );
        startup.finish();
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
