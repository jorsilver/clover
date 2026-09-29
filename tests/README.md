# Generator tests

Golden-file tests for the Blockly → Python code generator in `clover_blocks/www/`.

They run in Node with no browser, no ROS, no Gazebo and no drone. A saved
workspace goes in, generated Python comes out, and it is compared against a
checked-in expected result.

```bash
cd tests
npm install
npm test
```

## Layout

| Path | What it is |
|---|---|
| `harness.js` | Loads the vendored Blockly bundles and `blocks.js` / `python.js` into a Node VM context |
| `missions/*.xml` | Saved Blockly workspaces — the test inputs |
| `golden/*.py` | The Python each workspace is expected to generate |
| `run.js` | Runner; prints the first differing line on failure |

## Adding a test

1. Build the program in the Blockly UI and export the workspace XML.
2. Save it as `missions/<name>.xml`.
3. Run `npm run test:update` to record the current output as the golden file.
4. **Read the generated file before committing it.** `--update` records whatever
   the generator currently produces, correct or not. The golden file is only
   worth having if a human has confirmed it.

Block `id` attributes in the mission XML are set explicitly. Blockly emits a
`_b('<block id>')` highlight call before each statement, so without pinned ids
the generated output changes on every run and nothing can be compared.

## Why this exists

The generator is the part of this project most likely to break silently. A
change to one block's Python can alter the output of every mission that uses it,
and the only way to notice used to be flying the mission in simulation. These
tests turn that into a one-second check, which makes the generator safe to
refactor and new blocks cheap to add.

## Known gap

These tests prove the generator emits the Python you expect. They do not prove
that Python flies correctly — that still needs the simulator. A middle layer,
executing the generated search-pattern code under `pytest` to assert coverage
and ordering properties, would close most of the remaining distance.
