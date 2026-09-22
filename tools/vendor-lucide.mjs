import assert from "node:assert/strict";
import { copyFile, mkdir, readFile, writeFile } from "node:fs/promises";

const source = new URL("../node_modules/lucide-static/", import.meta.url);
const target = new URL("../assets/icons/lucide/", import.meta.url);
const icons = ["route", "car", "camera", "eye", "chart-no-axes-combined", "timer"];

await mkdir(target, { recursive: true });
for (const icon of icons) {
  const svg = await readFile(new URL(`icons/${icon}.svg`, source), "utf8");
  assert(svg.includes('stroke="currentColor"'), `${icon}: expected currentColor stroke`);
  // egui multiplies image colors by its tint; white preserves the chosen UI color.
  await writeFile(new URL(`${icon}.svg`, target), svg.replaceAll('stroke="currentColor"', 'stroke="white"'));
}
await copyFile(new URL("LICENSE", source), new URL("LICENSE", target));
