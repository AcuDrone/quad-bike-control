# PlatformIO pre-script: pack the web page gzip-compressed into the LittleFS image.
#
# Change reduce-web-heap-usage, design D4. Acts only when a filesystem image is built
# (buildfs / uploadfs / uploadfsota); every other target is untouched.
#
# data/ is staged into $BUILD_DIR/fsdata: every *.html is written as <name>.html.gz
# (gzip level 9, gzip mtime 0), every other file is copied unchanged; staged files keep the
# source file's mtime (mklittlefs stores it), so the same data/ gives a byte-identical image.
# PROJECT_DATA_DIR is then pointed at the staging directory, so the platform builder
# (main.py, env.DataToBin(..., "$PROJECT_DATA_DIR")) packs it. data/index.html stays the
# only source in git; the staging dir lies under .pio (git-ignored).
#
# ESPAsyncWebServer serves <path>.gz with Content-Encoding: gzip when <path> is absent,
# both for request->send(FS, "/index.html", ...) and for serveStatic (design D5).

import gzip
import os
import shutil

from SCons.Script import COMMAND_LINE_TARGETS  # pylint: disable=import-error

Import("env")  # noqa: F821  pylint: disable=undefined-variable

FS_TARGETS = {"buildfs", "uploadfs", "uploadfsota"}


def _fail(msg):
    raise RuntimeError("fs_gzip: " + msg)


def _stage(src_dir, dst_dir):
    if not os.path.isdir(src_dir):
        _fail("data directory not found: %s" % src_dir)
    entries = [e for e in os.listdir(src_dir) if not e.startswith(".")]
    if not entries:
        _fail("data directory is empty: %s" % src_dir)

    if os.path.isdir(dst_dir):
        shutil.rmtree(dst_dir)
    os.makedirs(dst_dir)

    expected = []
    for root, dirs, files in os.walk(src_dir):
        dirs[:] = sorted(d for d in dirs if not d.startswith("."))
        rel = os.path.relpath(root, src_dir)
        out_root = dst_dir if rel == "." else os.path.join(dst_dir, rel)
        os.makedirs(out_root, exist_ok=True)
        for name in sorted(files):
            if name.startswith("."):
                continue
            src = os.path.join(root, name)
            if name.endswith(".html"):
                dst = os.path.join(out_root, name + ".gz")
                with open(src, "rb") as f_in:
                    data = f_in.read()
                with open(dst, "wb") as f_raw:
                    with gzip.GzipFile(filename=name, mode="wb", fileobj=f_raw,
                                       compresslevel=9, mtime=0) as f_gz:
                        f_gz.write(data)
                # mklittlefs stores each file's mtime in the image: give the staged file the
                # source's mtime, so the same data/ gives a byte-identical littlefs.bin
                shutil.copystat(src, dst)
                print("fs_gzip: %s %d B -> %d B (staged in %s)"
                      % (os.path.relpath(src, src_dir), len(data),
                         os.path.getsize(dst), dst_dir))
            else:
                dst = os.path.join(out_root, name)
                shutil.copy2(src, dst)  # keeps the mtime (see above)
            expected.append(dst)

    if not expected:
        _fail("nothing staged from %s" % src_dir)
    for path in expected:
        if not os.path.isfile(path) or (path.endswith(".gz") and os.path.getsize(path) == 0):
            _fail("staged file missing or empty: %s" % path)


if FS_TARGETS & set(COMMAND_LINE_TARGETS):
    data_dir = env.subst("$PROJECT_DATA_DIR")  # noqa: F821
    stage_dir = os.path.join(env.subst("$BUILD_DIR"), "fsdata")  # noqa: F821
    _stage(data_dir, stage_dir)
    env.Replace(PROJECT_DATA_DIR=stage_dir)  # noqa: F821
