// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Runs a wasm32-wasi command module with node's WASI (preview1), the whole file system preopened at /, so arguments are
// absolute paths. Usage: node wasi-run.mjs module.wasm [args...]
import { readFileSync } from 'node:fs';
import { WASI } from 'node:wasi';

const [, , wasmPath, ...args] = process.argv;
const wasi = new WASI({ version: 'preview1', args: [wasmPath, ...args], env: {}, preopens: { '/': '/' } });
const module = await WebAssembly.compile(readFileSync(wasmPath));
const imports = wasi.getImportObject();
// The MSFS wasi-libc adds memory-management imports (commit_pages, ...) the sim provides; here memory is always
// committed, so they are no-ops returning 0.
for (const imp of WebAssembly.Module.imports(module)) {
    imports[imp.module] ??= {};
    if (imp.kind === 'function' && !(imp.name in imports[imp.module])) {
        imports[imp.module][imp.name] = () => 0;
        if (process.env.WASI_RUN_VERBOSE) {
            console.error(`stub import ${imp.module}.${imp.name}`);
        }
    }
}
const instance = await WebAssembly.instantiate(module, imports);
process.exitCode = wasi.start(instance);
