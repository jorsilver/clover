#!/usr/bin/env node
// Golden-file tests for the Blockly -> Python generator.
//
//   node tests/run.js            check every mission against its golden file
//   node tests/run.js --update   regenerate the golden files
//
// Each tests/missions/<name>.xml is a saved Blockly workspace. Its golden file
// is the Python the generator should produce for it. No ROS, no simulator.
const fs = require('fs');
const path = require('path');
const { generate } = require('./harness.js');

const MISSIONS = path.join(__dirname, 'missions');
const GOLDEN = path.join(__dirname, 'golden');
const update = process.argv.includes('--update');

function firstDiff(a, b) {
  const x = a.split('\n'), y = b.split('\n');
  for (let i = 0; i < Math.max(x.length, y.length); i++) {
    if (x[i] !== y[i]) {
      return `  line ${i + 1}\n    expected: ${JSON.stringify(x[i])}\n    actual:   ${JSON.stringify(y[i])}`;
    }
  }
  return '  (files differ only in trailing content)';
}

let failed = 0, passed = 0;
for (const file of fs.readdirSync(MISSIONS).filter(f => f.endsWith('.xml')).sort()) {
  const name = path.basename(file, '.xml');
  const actual = generate(fs.readFileSync(path.join(MISSIONS, file), 'utf8'));
  const goldenPath = path.join(GOLDEN, `${name}.py`);

  if (update || !fs.existsSync(goldenPath)) {
    fs.writeFileSync(goldenPath, actual);
    console.log(`~ ${name} (golden written)`);
    continue;
  }
  const expected = fs.readFileSync(goldenPath, 'utf8');
  if (expected === actual) {
    console.log(`✓ ${name}`);
    passed++;
  } else {
    console.log(`✗ ${name}\n${firstDiff(expected, actual)}`);
    failed++;
  }
}

if (!update) console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
