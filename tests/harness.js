// Headless loader for the clover_blocks Blockly generators.
//
// blocks.js and python.js are browser ES modules that expect a live Blockly
// workspace and ROS-supplied parameters. This loads them in a Node VM context
// instead, so generator output can be tested with no browser, no ROS and no
// simulator. Two source transforms are applied, both purely mechanical:
//   - the `import {params} from './ros.js'` line is dropped (params is injected)
//   - `export function` becomes `function` (exports are read off the sandbox)
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const { DOMParser, DOMImplementation } = require('@xmldom/xmldom');

// Blockly's XML helpers expect browser globals. These are the only two it uses.
if (typeof globalThis.DOMParser === 'undefined') globalThis.DOMParser = DOMParser;
if (typeof globalThis.document === 'undefined') {
  globalThis.document = new DOMImplementation().createDocument(null, null, null);
}

const WWW = path.resolve(__dirname, '..', 'clover_blocks', 'www');

// Defaults for the ROS parameters the generators read (see ros.js).
const DEFAULT_PARAMS = {
  navigate_tolerance: 0.2,
  navigate_global_tolerance: 1.0,
  yaw_tolerance: 0.1,
  sleep_time: 0.2,
};

function stripModuleSyntax(src) {
  return src
    .replace(/^\s*import\s+.*?;\s*$/gm, '')
    .replace(/^export\s+(function|const|let|var|class)\b/gm, '$1');
}

function load(params = DEFAULT_PARAMS) {
  const Blockly = require(path.join(WWW, 'blockly', 'blockly_compressed.js'));
  require(path.join(WWW, 'blockly', 'blocks_compressed.js'));
  require(path.join(WWW, 'blockly', 'python_compressed.js'));

  const sandbox = { Blockly, params, console };
  vm.createContext(sandbox);
  for (const f of ['blocks.js', 'python.js']) {
    const src = stripModuleSyntax(fs.readFileSync(path.join(WWW, f), 'utf8'));
    vm.runInContext(src, sandbox, { filename: f });
  }
  return sandbox;
}

function workspaceFromXml(Blockly, xml) {
  const workspace = new Blockly.Workspace();
  Blockly.Xml.domToWorkspace(Blockly.Xml.textToDom(xml), workspace);
  return workspace;
}

// Full runnable mission script, as the "Run" button produces it.
function generate(xml, params) {
  const sb = load(params);
  const ws = workspaceFromXml(sb.Blockly, xml);
  try { return sb.generateCode(ws); } finally { ws.dispose(); }
}

// Just the user's blocks, without the injected ROS preamble.
function generateUser(xml, params) {
  const sb = load(params);
  const ws = workspaceFromXml(sb.Blockly, xml);
  try { return sb.generateUserCode(ws); } finally { ws.dispose(); }
}

module.exports = { load, generate, generateUser, DEFAULT_PARAMS, WWW };
