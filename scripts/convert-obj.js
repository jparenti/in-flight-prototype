#!/usr/bin/env node
// Converts the source OBJ + texture into a Draco-compressed GLB in public/models/
// Usage: npm run convert

import { promises as fs } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import obj2gltf from 'obj2gltf';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const projectRoot = path.resolve(__dirname, '..');

const SRC_OBJ = '/Users/joshparenti/Downloads/Hive34-Release Testing - 3d Mesh Obj Zip/stallikon_new.obj';
const OUT_GLB = path.join(projectRoot, 'public/models/property.glb');

const options = {
  binary: true,
  separate: false,
  separateTextures: false,
  checkTransparency: false,
  secure: false,
  packOcclusion: false,
  metallicRoughness: false,
  baseColorTexture: true,
  unlit: true,
  dracoOptions: {
    compressionLevel: 7,
    quantizePositionBits: 14,
    quantizeNormalBits: 10,
    quantizeTexcoordBits: 12,
    quantizeColorBits: 8,
    quantizeGenericBits: 12,
    unifiedQuantization: false
  }
};

async function main() {
  console.log(`Reading: ${SRC_OBJ}`);
  const srcStat = await fs.stat(SRC_OBJ);
  console.log(`Source OBJ size: ${(srcStat.size / 1024 / 1024).toFixed(1)} MB`);

  console.log('Converting to Draco-compressed GLB (this may take a minute)…');
  const t0 = Date.now();
  const glb = await obj2gltf(SRC_OBJ, options);
  await fs.mkdir(path.dirname(OUT_GLB), { recursive: true });
  await fs.writeFile(OUT_GLB, glb);
  const ms = Date.now() - t0;
  const outStat = await fs.stat(OUT_GLB);
  console.log(`✓ Wrote ${OUT_GLB}`);
  console.log(`  Output size: ${(outStat.size / 1024 / 1024).toFixed(1)} MB`);
  console.log(`  Compression ratio: ${(srcStat.size / outStat.size).toFixed(1)}x`);
  console.log(`  Time: ${(ms / 1000).toFixed(1)}s`);
}

main().catch((err) => {
  console.error('Conversion failed:', err);
  process.exit(1);
});
