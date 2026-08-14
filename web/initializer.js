export default function initializer() {
  const startup = window.nanoplanStartup;
  let finishingStarted;
  let finishingTimer;
  let lastFinishingNote;
  const finishingNotes = [
    "Download complete. The browser is finishing WebAssembly compilation.",
    "Linking the motion planners, simulation, and 2D renderer.",
    "Instantiating the optimized module and preparing its memory.",
  ];
  const showFinishing = () => {
    const elapsed = performance.now() - finishingStarted;
    const note = finishingNotes[Math.floor(elapsed / 4000) % finishingNotes.length];
    if (note !== lastFinishingNote) {
      startup.show("cog", "Finishing the optimized driving engine", note);
      lastFinishingNote = note;
    }
    startup.transfer(Math.floor(elapsed / 1000) + " s of browser CPU work elapsed");
  };

  return {
    onStart: () =>
      startup.show("download", "Loading the driving engine", "Opening the WebAssembly stream."),
    onProgress: ({ current, total }) => {
      if (!total) return;
      if (current < total) {
        startup.show(
          "download",
          "Loading the driving engine",
          "Streaming WebAssembly while the browser begins compiling it.",
        );
        startup.transfer((current / 1_000_000).toFixed(1) + " MB received");
      } else if (!finishingTimer) {
        startup.transfer("");
        finishingStarted = performance.now();
        showFinishing();
        finishingTimer = setInterval(showFinishing, 1000);
      }
    },
    onSuccess: () => {
      clearInterval(finishingTimer);
      startup.transfer("");
      performance.mark("nanoplan-wasm-ready");
      startup.show(
        "app-window",
        "Driving engine ready",
        "Opening the renderer, controls, starter circuit, and first interactive frame.",
      );
    },
    onFailure: (error) => {
      clearInterval(finishingTimer);
      startup.transfer("");
      startup.show(
        "triangle-alert",
        "Nanoplan failed to start",
        "The driving engine could not be compiled or initialized.",
      );
      console.error(error);
    },
  };
}
