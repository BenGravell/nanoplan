import { copyFile, mkdir } from "node:fs/promises";
import { join } from "node:path";

// Browse https://lucide.dev/icons/
// Each name selects lucide-static/icons/<name>.svg.
const icons = [
  "app-window",
  "circle-check",
  "cog",
  "download",
  "flag",
  "maximize",
  "refresh-cw",
  "save",
  "triangle-alert",
];
const source = join(process.env.TRUNK_SOURCE_DIR, "node_modules/lucide-static/icons");
const target = join(process.env.TRUNK_STAGING_DIR, "icons");

await mkdir(target, { recursive: true });
await Promise.all(icons.map((icon) => copyFile(join(source, `${icon}.svg`), join(target, `${icon}.svg`))));
