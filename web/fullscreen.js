const button = document.querySelector("#fullscreen");
const enterFullscreen = () => {
  if (!document.fullscreenElement)
    document.documentElement.requestFullscreen?.().catch(() => {});
};

button.addEventListener("click", enterFullscreen);
document.addEventListener("keydown", enterFullscreen, { once: true });
document.addEventListener("fullscreenchange", () => {
  button.hidden = Boolean(document.fullscreenElement);
});
if (!document.documentElement.requestFullscreen) button.hidden = true;
