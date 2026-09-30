// R53 flasher, for the R53 Shift Light's C3 board and its K-line + CAN board.
// Talks to the board over Web Serial with esptool-js, works out which board it
// is from the chip and which published build it is running from its flash, and
// writes a build listed in releases.json (made by publish-release.ps1).

// Pinned to an exact version: a flasher that changes under us between two
// visits is the last thing that should happen to a board.
import { CustomReset, ESPLoader, Transport } from "https://cdn.jsdelivr.net/npm/esptool-js@0.7.0/bundle.js";

const BAUD_FLASH = 921600;
const BAUD_ROM = 115200;

// Pulse EN with the boot strap (GPIO9 on the C3, GPIO0 on the S3) left high, so
// the chip boots its firmware. Both boards use the chip's own USB-Serial/JTAG,
// where RTS drives EN and DTR the strap. esptool-js 0.7.0's HardReset only
// drops RTS, which is already low after its bootloader reset, so on its own it
// resets nothing and the board stays in the ROM bootloader.
const RUN_FIRMWARE_RESET = "D0|R1|W200|R0|W200";

// Keyed by PlatformIO env, which is also the folder name under firmware/<build>/.
const BOARDS = {
  "esp32-c3": {
    name: "R53 Shift Light C3 board",
    short: "C3",
    module: "ESP32-C3 SuperMini",
    chip: "ESP32-C3",
    flashMB: 4,
    spec: "ESP32-C3 · 4 MB flash",
    blurb: "The 54 × 58 mm shift light board with the C3 SuperMini soldered on. A bare SuperMini takes the same build.",
    ledText: "the blue LED on the SuperMini blinks once a second",
    keepsText: "Thresholds, colours and brightness you saved stay as they are.",
    freshText: "It's restarting on the default settings. Set it up in the app and press Save.",
    art: moduleArt("C3", "#1b3a8a"),
  },
  // The rev C K-line + CAN board. Firmware from R53_Mini_Kline_Canbus_Logger_Shiftlight.
  "xiao_esp32s3": {
    name: "R53 Shift Light K-line + CAN board",
    short: "K-line + CAN",
    module: "Seeed XIAO ESP32-S3",
    chip: "ESP32-S3",
    flashMB: 8,
    spec: "ESP32-S3 · 8 MB flash · 8 MB PSRAM",
    blurb: "Rev C, with the XIAO soldered flat: K-line, CAN, wideband and the shift light on one board.",
    ledText: "the orange LED on the XIAO blinks once a second",
    keepsText: "The K-line polling set and the shift light settings you saved stay as they are.",
    freshText: "It's restarting on the defaults. Set up the shift light in the R53 Shift app and the K-line polling from R53 Logger.",
    art: moduleArt("S3", "#2b2f36"),
  },
};

// How many published builds Connect checks the board's app against. Each is one
// md5 of the app region, well under a second with the stub.
const RUNNING_CHECK_MAX = 6;

// The C3 SuperMini only comes as the ESP32-C3FH4, and the XIAO ESP32-S3 is an
// ESP32-S3R8: 8 MB flash, 8 MB PSRAM. Anything else is some other board and
// gets refused, unless someone picks by hand. The S3-Zero prototype (4 MB, 2 MB
// PSRAM) was retired on 2026-09-29; build 1 in releases.json still carries it.
function boardForChip(chip) {
  if (chip.name === "ESP32-C3" && chip.flashMB === 4) return "esp32-c3";
  if (chip.name === "ESP32-S3" && chip.flashMB === 8 && chip.psramMB === 8) return "xiao_esp32s3";
  return null;
}

// ---------------------------------------------------------------- state

const state = {
  manifest: null,
  release: null,
  boardKey: null,
  pickedManually: false,
  chip: null,          // { name, flashMB, psramMB, mac, tableMd5 } once read
  port: null,
  portInfo: null,      // USB IDs of the last port picked, to find it again for the flash
  transport: null,
  loader: null,
  busy: false,
  writing: false,      // erasing or writing flash: never reset the chip then
  flashedMacs: [],     // every board flashed since the page opened, for a batch
};

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------- log

const logEl = $("log");
function log(line) {
  logEl.textContent += line + "\n";
  logEl.scrollTop = logEl.scrollHeight;
}
const terminal = {
  clean() {},
  writeLine(data) { log(data); },
  write(data) { logEl.textContent += data; },
};

// ---------------------------------------------------------------- serial

// The port picked at Connect, found again for the flash so the user isn't
// asked twice. Only when exactly one granted port has those USB IDs: every
// C3 and S3 shows up as 303a:1001, and the board re-enumerates when it resets,
// so the old SerialPort object can't be kept.
async function findPort() {
  if (state.portInfo) {
    const granted = await navigator.serial.getPorts();
    const same = granted.filter((port) => {
      const info = port.getInfo();
      return info.usbVendorId === state.portInfo.usbVendorId && info.usbProductId === state.portInfo.usbProductId;
    });
    if (same.length === 1) return same[0];
  }
  // No USB filter: the chip check decides what this is, not the USB IDs.
  return navigator.serial.requestPort();
}

async function openLoader() {
  if (state.loader) return state.loader;
  state.port = await findPort();
  state.portInfo = state.port.getInfo();
  state.transport = new Transport(state.port, false);
  state.loader = new ESPLoader({
    transport: state.transport,
    baudrate: BAUD_FLASH,
    romBaudrate: BAUD_ROM,
    terminal,
  });
  try {
    // Resets into the ROM bootloader (esptool-js uses the USB-Serial/JTAG
    // sequence on its own for 303a:1001), identifies the chip, loads the stub.
    await state.loader.main();
  } catch (err) {
    await releaseBoard();
    throw err;
  }
  state.port.addEventListener("disconnect", onUnplugged);
  return state.loader;
}

// Resets the board back into its firmware and closes the port. main() leaves
// the chip in the ROM bootloader until something resets it, so every path that
// opened the port ends here.
async function releaseBoard() {
  const transport = state.transport;
  // Unhooked first: the board drops off USB while it resets, and that isn't
  // the user unplugging it.
  if (state.port) state.port.removeEventListener("disconnect", onUnplugged);
  state.loader = null;
  state.transport = null;
  state.port = null;
  if (!transport) return;
  try {
    await new CustomReset(transport, RUN_FIRMWARE_RESET).reset();
    log("Board restarted into its firmware.");
  } catch { /* port never opened, or already gone */ }
  try { await transport.disconnect(); } catch { /* already gone */ }
}

function onUnplugged() {
  state.loader = null;
  state.transport = null;
  state.port = null;
  state.portInfo = null;
  state.chip = null;
  if (!state.pickedManually) state.boardKey = null;
  log("Board unplugged.");
  render();
}

async function readChip(loader) {
  const flashSize = await loader.detectFlashSize();           // "4MB", or undefined
  const features = await loader.chip.getChipFeatures(loader);  // the S3 lists "Embedded PSRAM 2MB (AP_3v3)"
  const psram = features.join(" ").match(/Embedded PSRAM (\d+)MB/);
  return {
    name: loader.chip.CHIP_NAME,
    flashMB: flashSize ? parseInt(flashSize, 10) : 0,
    psramMB: psram ? parseInt(psram[1], 10) : 0,
    mac: await loader.chip.readMac(loader),
    tableMd5: null,
    runningBuild: null,  // the published build whose app is on the flash, if any
  };
}

// Which published build's app is on the board, by the md5 of each build's app
// where the app lives. That is what tells an upgrade from a reinstall. Only
// asked when the partition table is this firmware's: another layout cannot be
// running one of these builds.
async function runningBuild(loader, boardKey) {
  if (!state.manifest) return null;
  const releases = state.manifest.releases.filter((release) => release.boards[boardKey]);
  for (const release of releases.slice(0, RUNNING_CHECK_MAX)) {
    const app = release.boards[boardKey].update.find((file) => file.what === "app");
    if (!app) continue;
    const onFlash = await loader.flashMd5sum(parseInt(app.offset, 16), app.size);
    if (onFlash === app.md5) return release.build;
  }
  return null;
}

// The newest build with an image for this board.
function latestBuildFor(boardKey) {
  const release = state.manifest && state.manifest.releases.find((r) => r.boards[boardKey]);
  return release ? release.build : null;
}

// Show a build that has an image for this board: the newest one, unless the
// build already showing has one.
function selectReleaseFor(boardKey) {
  if (!state.manifest || !boardKey) return;
  if (state.release && state.release.boards[boardKey]) return;
  const i = state.manifest.releases.findIndex((release) => release.boards[boardKey]);
  if (i < 0) return;
  state.release = state.manifest.releases[i];
  $("release-select").value = String(i);
}

// ---------------------------------------------------------------- step 1: device

async function connectAndDetect() {
  if (state.busy) return;
  state.busy = true;
  $("device-error").hidden = true;
  render();
  try {
    await releaseBoard();
    // Connect always offers the port list, in case it's a different board.
    state.portInfo = null;
    const loader = await openLoader();
    state.chip = await readChip(loader);
    const detected = boardForChip(state.chip);
    selectReleaseFor(detected);
    // Read the partition table while the port is open anyway, so the flash
    // dialog can offer Update only to a board already laid out for this build,
    // and on such a board find which build it is running.
    const table = tableFor(detected);
    if (table) state.chip.tableMd5 = await loader.flashMd5sum(parseInt(table.offset, 16), table.size);
    if (table && state.chip.tableMd5 === table.md5) {
      state.chip.runningBuild = await runningBuild(loader, detected);
      log(`Running build: ${state.chip.runningBuild || "not a published one"}`);
    }
    // Detection only needs the bootloader for a moment. Flashing reconnects,
    // which resets it back into the bootloader.
    await releaseBoard();
    log(`Chip ${chipText(state.chip)} -> ${detected || "unknown"}`);
    if (detected) {
      state.boardKey = detected;
      state.pickedManually = false;
    } else if (!state.pickedManually) {
      state.boardKey = null;
      showDeviceError(`Not a board this page knows (${chipText(state.chip)}). Nothing flashed. If you're sure what it is, use "I know what this is".`);
    }
  } catch (err) {
    if (err && err.name === "NotFoundError") {
      log("No port picked.");
    } else {
      showDeviceError(`Couldn't talk to the board: ${messageOf(err)}. See "If it won't connect" below.`);
      log(String(err && err.stack || err));
    }
    await releaseBoard();
  } finally {
    state.busy = false;
    render();
  }
}

function tableFor(boardKey) {
  const plan = boardKey && state.release && state.release.boards[boardKey];
  return plan ? plan.partitionTable : null;
}

function showDeviceError(text) {
  $("device-error").textContent = text;
  $("device-error").hidden = false;
}

function chipText(chip) {
  return `${chip.name}, ${chip.flashMB || "?"} MB flash, ${chip.psramMB ? chip.psramMB + " MB" : "no"} PSRAM`;
}

function openDevicePicker() {
  const grid = $("device-grid");
  grid.textContent = "";
  for (const [key, board] of Object.entries(BOARDS)) {
    const card = document.createElement("button");
    card.type = "button";
    card.className = "device-card";
    card.innerHTML = `${board.art}
      <span class="name">${board.name}</span>
      <span class="module">${board.module}</span>
      <span class="spec">${board.spec}</span>
      <span class="blurb">${board.blurb}</span>`;
    card.addEventListener("click", () => {
      state.boardKey = key;
      state.pickedManually = true;
      selectReleaseFor(key);
      $("device-error").hidden = true;
      $("device-dialog").close();
      render();
    });
    grid.appendChild(card);
  }
  $("device-dialog").showModal();
}

// ---------------------------------------------------------------- step 2: firmware

async function loadReleases() {
  const select = $("release-select");
  try {
    const response = await fetch("releases.json", { cache: "no-cache" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    state.manifest = await response.json();
  } catch (err) {
    select.innerHTML = "<option>No builds published</option>";
    $("release-detail").textContent = `Couldn't load releases.json (${messageOf(err)}).`;
    return;
  }
  // Newest build first, and the newest is the default.
  const releases = [...state.manifest.releases].sort((a, b) => b.build - a.build);
  state.manifest.releases = releases;
  select.textContent = "";
  releases.forEach((release, i) => {
    const option = document.createElement("option");
    option.value = String(i);
    // Which boards each build is for, since a build need not carry both.
    const boards = Object.keys(release.boards).filter((key) => BOARDS[key]).map((key) => BOARDS[key].short);
    option.textContent = `Build ${release.build}${i === 0 ? " (latest)" : ""} · ${boards.join(" + ")} · ${release.date}${release.bench ? " · bench" : ""}`;
    select.appendChild(option);
  });
  select.disabled = releases.length === 0;
  state.release = releases[0] || null;
  selectReleaseFor(state.boardKey);
  select.addEventListener("change", () => {
    state.release = state.manifest.releases[Number(select.value)];
    render();
  });
  render();
}

// ---------------------------------------------------------------- step 3: flash

// "match", "other", or null when the table wasn't read (picked by hand, or the
// builds hadn't loaded at Connect). Update checks it again before writing.
function layoutOnBoard() {
  const table = tableFor(state.boardKey);
  if (!state.chip || !state.chip.tableMd5 || !table) return null;
  return state.chip.tableMd5 === table.md5 ? "match" : "other";
}

function openFlashDialog() {
  const board = BOARDS[state.boardKey];
  const release = state.release;
  $("flash-art").innerHTML = board.art;
  $("flash-what").innerHTML = `<strong>${board.name}</strong> (${board.module}), build ${release.build}<br>
    <span class="note">${state.pickedManually ? "Board picked by hand" : "Detected from the chip"}</span>`;
  $("progress-wrap").hidden = true;
  $("result").hidden = true;
  $("btn-next").hidden = true;

  // A new board, or one running something else, has no layout Update can
  // write into, so it goes straight to Fresh install.
  const needsFresh = layoutOnBoard() === "other";
  const updateInput = $("flash-choices").querySelector("input[value=update]");
  $("flash-choices").querySelectorAll("input").forEach((input) => { input.disabled = false; });
  updateInput.disabled = needsFresh;
  $("choice-update").classList.toggle("choice-off", needsFresh);
  $("update-desc").textContent = needsFresh
    ? "Not available: this board doesn't have this firmware's layout on it yet (a new board, or other firmware). Use Fresh install."
    : `Writes the new firmware only. ${board.keepsText}`;
  $("flash-choices").querySelector(`input[value=${needsFresh ? "fresh" : "update"}]`).checked = true;
  syncStartButton();
  $("btn-start").disabled = false;
  $("btn-start").hidden = false;
  $("flash-dialog").showModal();
}

function selectedMode() {
  return $("flash-choices").querySelector("input:checked").value;
}

function syncStartButton() {
  const fresh = selectedMode() === "fresh";
  const button = $("btn-start");
  button.textContent = fresh ? "Erase and install" : "Update now";
  button.classList.toggle("btn-danger", fresh);
  button.classList.toggle("btn-primary", !fresh);
}

function setStage(text, fraction) {
  $("progress-wrap").hidden = false;
  $("progress-stage").textContent = text;
  const indeterminate = fraction === null;
  $("progress").classList.toggle("indeterminate", indeterminate);
  $("progress-bar").style.width = indeterminate ? "" : `${Math.round(fraction * 100)}%`;
  $("progress-pct").textContent = indeterminate ? "" : `${Math.round(fraction * 100)}%`;
}

function showResult(ok, html) {
  const box = $("result");
  box.className = `result ${ok ? "result-ok" : "result-bad"}`;
  box.innerHTML = html;
  box.hidden = false;
}

// Refusals that stop before anything is written. Thrown, caught in startFlash
// and shown as "nothing flashed".
class Refusal extends Error {}

// nextBoard: the "Flash the next board" button after a good flash. It asks for
// the port again (each C3 has its own USB serial number, so Chrome treats it
// as a new device), detects the chip, and flashes in the same mode.
async function startFlash(nextBoard = false) {
  if (state.busy) return;
  state.busy = true;
  const mode = selectedMode();
  const release = state.release;
  let wroteSomething = false;
  $("btn-start").disabled = true;
  $("btn-next").hidden = true;
  $("flash-choices").querySelectorAll("input").forEach((input) => { input.disabled = true; });
  $("result").hidden = true;
  if (nextBoard) {
    state.portInfo = null;
    state.chip = null;
    if (!state.pickedManually) state.boardKey = null;
  }
  render();

  try {
    setStage("Connecting", null);
    const loader = await openLoader();
    state.chip = await readChip(loader);
    if (nextBoard && !state.pickedManually) {
      state.boardKey = boardForChip(state.chip);
      if (!state.boardKey) throw new Refusal(`Not a board this page knows (${chipText(state.chip)}).`);
    }
    const boardKey = state.boardKey;
    const board = BOARDS[boardKey];
    const plan = release.boards[boardKey];
    if (!plan) throw new Refusal(`Build ${release.build} has no image for the ${board.name}.`);
    checkChipAgainstPick(boardKey);

    // Update: the app at 0x10000, then otadata at 0xe000 pointing the
    // bootloader at app0 (publish-release.ps1 has why). NVS at 0x9000, which
    // holds the saved config, is not in the list, so it survives.
    // Fresh: erase everything, then bootloader, partition table, otadata, app.
    const files = mode === "fresh" ? plan.fresh : plan.update;

    if (mode === "update") {
      setStage("Checking the partition table", null);
      await checkPartitionTable(loader, plan.partitionTable);
    }

    // Download and check everything before touching the flash, so a bad
    // download can never leave a half-written board.
    const images = [];
    for (const [i, file] of files.entries()) {
      setStage(`Downloading ${shortName(file)}`, i / files.length);
      images.push(await download(file));
    }

    state.writing = true;
    if (mode === "fresh") {
      setStage("Erasing the flash", null);
      wroteSomething = true;
      await loader.eraseFlash();
    }

    const totalBytes = images.reduce((sum, image) => sum + image.data.length, 0);
    const doneBefore = images.map((_, i) => images.slice(0, i).reduce((sum, image) => sum + image.data.length, 0));
    setStage("Writing", 0);
    wroteSomething = true;
    await loader.writeFlash({
      fileArray: images.map((image) => ({ data: image.data, address: image.address })),
      flashMode: "keep",
      flashFreq: "keep",
      flashSize: "keep",
      eraseAll: false,
      compress: true,
      reportProgress(fileIndex, written, total) {
        const fileFraction = total ? written / total : 1;
        const overall = (doneBefore[fileIndex] + fileFraction * images[fileIndex].data.length) / totalBytes;
        setStage(`Writing ${shortName(files[fileIndex])}`, overall);
      },
    });
    state.writing = false;

    for (const [i, image] of images.entries()) {
      setStage(`Verifying ${shortName(files[i])}`, null);
      const onFlash = await loader.flashMd5sum(image.address, image.data.length);
      log(`md5 at 0x${image.address.toString(16)}: flash ${onFlash}, expected ${files[i].md5}`);
      if (onFlash !== files[i].md5) {
        throw new Error(`What's on the flash at 0x${image.address.toString(16)} doesn't match the ${shortName(files[i])}. Flash again; if it keeps failing, try another cable or USB port.`);
      }
    }

    const mac = state.chip.mac;
    setStage("Restarting the board", 1);
    await releaseBoard();
    state.chip = null;
    if (!state.flashedMacs.includes(mac)) state.flashedMacs.push(mac);
    log(`Flashed ${mac}, build ${release.build}, ${mode}. Boards flashed on this page: ${state.flashedMacs.length}`);
    setStage("Done", 1);
    showResult(true, mode === "fresh"
      ? `Build ${release.build} installed and verified on the ${escapeHtml(board.name.toLowerCase())}, <code>${escapeHtml(mac)}</code>.
         ${escapeHtml(board.freshText)} Away from the car ${escapeHtml(board.ledText)}.`
      : `Build ${release.build} written and verified on <code>${escapeHtml(mac)}</code>. Its saved settings are kept. Away from the car ${escapeHtml(board.ledText)}.`);
    $("btn-start").hidden = true;
    $("btn-next").hidden = false;
    $("btn-next").textContent = `Flash the next board (${state.flashedMacs.length} done)`;
  } catch (err) {
    log(String(err && err.stack || err));
    const nothing = err instanceof Refusal || !wroteSomething;
    const reason = messageOf(err);
    if (err && err.name === "NotFoundError") {
      showResult(false, "Nothing flashed. No port picked.");
    } else {
      showResult(false, nothing
        ? `Nothing flashed. ${escapeHtml(reason)}`
        : `Flashing failed partway: ${escapeHtml(reason)} The board may not start until it's flashed again. Unplug it, plug it back in and run it again; if it won't connect, hold BOOT while plugging it in.`);
    }
    setStage(nothing ? "Stopped" : "Failed", null);
    $("progress").classList.remove("indeterminate");
    state.writing = false;
    // Reset even after a failed write: a half-written board is no worse off
    // booting, and a refused one comes straight back up on its old firmware.
    await releaseBoard();
  } finally {
    state.busy = false;
    $("flash-choices").querySelectorAll("input").forEach((input) => { input.disabled = false; });
    // Update stays off for a board known not to have the layout.
    if (layoutOnBoard() === "other") $("flash-choices").querySelector("input[value=update]").disabled = true;
    $("btn-start").disabled = false;
    // Once a batch has started, Start is hidden; a failed board in the batch
    // retries from the next-board button, which asks for the port again.
    if ($("btn-start").hidden) $("btn-next").hidden = false;
    render();
  }
}

// The chip is read again right before writing. An automatic pick has to still
// match; a hand pick is trusted, except that the image must be for this chip
// family and fit in its flash.
function checkChipAgainstPick(boardKey) {
  const board = BOARDS[boardKey];
  const detected = boardForChip(state.chip);
  log(`Chip now: ${chipText(state.chip)}`);
  if (state.chip.name !== board.chip) {
    throw new Refusal(`This is an ${state.chip.name}, and the ${board.name} image is for an ${board.chip}.`);
  }
  // Unknown size (readChip() turns an undetected size into 0) stops here too:
  // it is the one check between a hand pick and an erase, and "unknown" is not
  // "big enough" (BUGS.md F1).
  if (!state.chip.flashMB) {
    throw new Refusal(`Couldn't read how much flash this chip has, so there's no telling whether the ${board.name} image fits. Unplug the board, plug it back in and press Connect again.`);
  }
  if (state.chip.flashMB < board.flashMB) {
    throw new Refusal(`This chip has ${state.chip.flashMB} MB of flash and the ${board.name} image needs ${board.flashMB} MB.`);
  }
  if (!state.pickedManually && detected !== boardKey) {
    throw new Refusal(detected
      ? `The board on this port is the ${BOARDS[detected].name.toLowerCase()} (${BOARDS[detected].module}), not the ${board.name.toLowerCase()} picked earlier. Press Connect again.`
      : `The board on this port isn't one this page knows (${chipText(state.chip)}).`);
  }
}

// Update writes the app into the existing layout, so the layout on the board
// has to be the one this build was linked for. The table sits at 0x8000; an
// md5 of it is a single cheap command. A board that fails this needs a fresh
// install, which writes the table.
async function checkPartitionTable(loader, table) {
  const onFlash = await loader.flashMd5sum(parseInt(table.offset, 16), table.size);
  log(`Partition table md5: flash ${onFlash}, build ${table.md5}`);
  if (state.chip) state.chip.tableMd5 = onFlash;
  if (onFlash !== table.md5) {
    throw new Refusal("This board doesn't have this firmware's partition layout, so an update could leave it unbootable. Use Fresh install instead.");
  }
}

async function download(file) {
  const response = await fetch(file.path);
  if (!response.ok) throw new Refusal(`Couldn't download ${file.path} (HTTP ${response.status}).`);
  const data = new Uint8Array(await response.arrayBuffer());
  if (data.length !== file.size) {
    throw new Refusal(`${file.path} is ${data.length} bytes, the manifest says ${file.size}.`);
  }
  // The stub only does md5, which checks the flash afterwards. sha256 is what
  // the browser can do, so it checks the download before any write.
  const digest = new Uint8Array(await crypto.subtle.digest("SHA-256", data));
  const sha256 = [...digest].map((b) => b.toString(16).padStart(2, "0")).join("");
  if (sha256 !== file.sha256) throw new Refusal(`${file.path} failed its checksum. Reload the page and try again.`);
  log(`Downloaded ${file.path}, ${data.length} bytes, sha256 ok`);
  return { data, address: parseInt(file.offset, 16) };
}

function shortName(file) {
  return { app: "firmware", otadata: "boot selector", bootloader: "bootloader", partitions: "partition table" }[file.what] || file.what;
}

// ---------------------------------------------------------------- render

function render() {
  const board = state.boardKey ? BOARDS[state.boardKey] : null;
  const release = state.release;
  // The port is closed again once the chip is read, so "connected" means a
  // board has been read, not that the port is open.
  const connected = !!state.chip;

  $("step-device").classList.toggle("ready", !!board);
  $("device-picked").hidden = !board;
  if (board) {
    $("device-picked-art").innerHTML = board.art;
    $("device-picked-name").textContent = `${board.name} · ${board.module}`;
    $("device-picked-how").textContent = state.pickedManually ? "Picked by hand" : "Detected from the chip";
    $("device-hint").hidden = true;
  } else {
    $("device-hint").hidden = false;
  }
  $("chip-line").hidden = !state.chip;
  if (state.chip) $("chip-line").textContent = `${chipText(state.chip)} · ${state.chip.mac}`;
  const layout = layoutOnBoard();
  const running = state.chip && state.chip.runningBuild;
  const latest = board ? latestBuildFor(state.boardKey) : null;
  $("layout-line").hidden = !layout;
  $("layout-line").textContent = layout === "other"
    ? "New board or other firmware on it: it needs a fresh install."
    : !running
      ? "Has this firmware's layout, running a build this page doesn't list. Update keeps its settings."
      : running === latest
        ? `Running build ${running}, the latest. Nothing to upgrade; Update would reinstall it and keep its settings.`
        : `Running build ${running}. Build ${latest} is newer, and Update keeps its settings.`;
  $("btn-connect").disabled = state.busy || !("serial" in navigator);
  $("btn-connect").textContent = state.busy && !$("flash-dialog").open ? "Connecting..." : connected ? "Connect again" : "Connect";

  $("step-firmware").classList.toggle("ready", !!release);
  $("bench-banner").hidden = !(release && release.bench);
  if (release) {
    // Each board's firmware can come from its own repo. A board entry that
    // names its own source (builds from 2 on) is taken as it stands, including
    // a null repo, which is the bridge's private one: it gets its commit and no
    // link. Builds before that carry only the release-level shift light commit.
    const plan = board && release.boards[state.boardKey];
    const perBoard = !!plan && "sourceCommit" in plan;
    const commit = perBoard ? plan.sourceCommit : release.sourceCommit;
    const repo = perBoard ? plan.sourceRepo : state.manifest.sourceRepo;
    $("release-detail").textContent = (release.notes || "") +
      (plan && plan.firmwareBuild ? ` K-line + CAN firmware ${plan.firmwareBuild}.` : "");
    const sourceHtml = !commit
      ? "Bench build, not from a published commit."
      : repo
        ? `Source: <a href="${repo}/tree/${commit}" target="_blank" rel="noopener">${escapeHtml(repo.split("/").pop())}@${commit.slice(0, 7)}</a>`
        : `Source: commit ${commit.slice(0, 7)}, in a private repo.`;
    $("release-source").innerHTML = sourceHtml;
    if (board && !release.boards[state.boardKey]) {
      $("release-detail").textContent += ` Not built for the ${board.name.toLowerCase()}.`;
    }
  }

  const canFlash = !!board && !!release && !!release.boards[state.boardKey] && ("serial" in navigator) && !state.busy;
  $("btn-flash").disabled = !canFlash;
  $("step-flash").classList.toggle("ready", canFlash);
  $("flash-hint").textContent = canFlash
    ? `Build ${release.build} for the ${board.name.toLowerCase()}. You'll choose between Update and Fresh install next.`
    : "Pick a board and a build first.";
}

// ---------------------------------------------------------------- helpers

function messageOf(err) {
  return (err && err.message) || String(err);
}

function escapeHtml(text) {
  return String(text).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
}

// A plain drawing of a small castellated module: USB-C at the top, pads down
// both edges, the chip in the middle. Not a photo and not anyone's artwork.
function moduleArt(label, pcb) {
  const pads = [0, 1, 2, 3, 4, 5, 6, 7].map((i) => {
    const y = 34 + i * 8;
    return `<rect x="30" y="${y}" width="6" height="4" rx="1" fill="#d4a017"/>
      <rect x="84" y="${y}" width="6" height="4" rx="1" fill="#d4a017"/>`;
  }).join("");
  return `<svg viewBox="0 0 120 110" role="img" aria-label="${label} module">
    <rect x="30" y="18" width="60" height="84" rx="5" fill="${pcb}" stroke="#4b5563" stroke-width="1.5"/>
    <rect x="48" y="8" width="24" height="16" rx="4" fill="#9ca3af"/>
    ${pads}
    <rect x="46" y="48" width="28" height="28" rx="2" fill="#0b0f14" stroke="#374151"/>
    <text x="60" y="67" text-anchor="middle" font-family="IBM Plex Mono, monospace" font-size="12" font-weight="500" fill="#FFB000">${label}</text>
    <rect x="42" y="84" width="10" height="6" rx="1.5" fill="#e5e7eb"/>
    <rect x="68" y="84" width="10" height="6" rx="1.5" fill="#e5e7eb"/>
  </svg>`;
}

// ---------------------------------------------------------------- wire up

function init() {
  if (!("serial" in navigator)) $("no-serial").hidden = false;
  $("btn-connect").addEventListener("click", connectAndDetect);
  $("btn-manual").addEventListener("click", openDevicePicker);
  $("btn-flash").addEventListener("click", openFlashDialog);
  $("btn-start").addEventListener("click", () => startFlash(false));
  $("btn-next").addEventListener("click", () => startFlash(true));
  $("flash-choices").addEventListener("change", syncStartButton);
  // The flash dialog stays open while busy: closing it mid-write would hide
  // the only progress there is.
  document.querySelectorAll("dialog [data-close]").forEach((button) => {
    button.addEventListener("click", () => {
      const dialog = button.closest("dialog");
      if (dialog.id === "flash-dialog" && state.busy) return;
      dialog.close();
    });
  });
  $("flash-dialog").addEventListener("cancel", (event) => { if (state.busy) event.preventDefault(); });
  // Nothing should still hold the port once the dialog closes, but if it does,
  // hand the board back rather than leave it in the bootloader.
  $("flash-dialog").addEventListener("close", () => { if (!state.busy) releaseBoard(); });
  // Best effort on the way out: the page may be gone before the reset lands.
  // Never mid-write, where a reset would boot a half-written image.
  window.addEventListener("pagehide", () => { if (!state.writing) releaseBoard(); });
  render();
  loadReleases();
}

init();
