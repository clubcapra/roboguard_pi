#!/usr/bin/env python3
import os
import subprocess
import shutil
import yaml
import sys

def dir_size(path) -> int:
    total = 0
    for dirpath, _, filenames in os.walk(path):
        for f in filenames:
            fp = os.path.join(dirpath, f)
            if os.path.isfile(fp):
                total += os.path.getsize(fp)
    return total

def fmt_size(size_bytes) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if size_bytes < 1024:
            return f"{size_bytes:.2f} {unit}"
        size_bytes /= 1024
    return f"{size_bytes:.2f} TB"

def find_bags(log_root) -> list:
    """Find all directories containing at least one .mcap file."""
    bags = []
    for dirpath, dirnames, filenames in os.walk(log_root):
        if any(f.endswith(".mcap") for f in filenames):
            bags.append(dirpath)
            dirnames.clear()
    return sorted(bags)

def is_compressed(bag_dir) -> bool:
    return bag_dir.endswith("_compressed")

def compress_bag(bag_dir):
    ros_log_dir = os.path.dirname(bag_dir)
    compressed_dir = bag_dir + "_compressed"
    config_path = os.path.join(ros_log_dir, "compress_config.yaml")

    convert_config = {
        "output_bags": [{
            "uri": compressed_dir,
            "storage_id": "mcap",
            "storage_preset_profile": "zstd_small",
            "all": True,
        }]
    }

    with open(config_path, "w") as f:
        yaml.dump(convert_config, f)

    original_size = dir_size(bag_dir)
    print(f"  Compressing ({fmt_size(original_size)})...")

    try:
        subprocess.run(
            ["ros2", "bag", "convert", "--input", bag_dir, "--output-options", config_path],
            check=True,
            timeout=600,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )

        if not os.path.isdir(compressed_dir) or not any(
            f.endswith(".mcap") for f in os.listdir(compressed_dir)
        ):
            print(f"  ERROR: output bag missing or empty — original preserved.")
            shutil.rmtree(compressed_dir, ignore_errors=True)
            return False

        backup_dir = bag_dir + "_backup"
        os.rename(bag_dir, backup_dir)
        os.rename(compressed_dir, bag_dir)
        shutil.rmtree(backup_dir)

        compressed_size = dir_size(bag_dir)
        ratio = original_size / compressed_size if compressed_size > 0 else 0
        saved = original_size - compressed_size
        print(
            f"  Done. "
            f"{fmt_size(original_size)} → {fmt_size(compressed_size)}, "
            f"saved {fmt_size(saved)} ({100 * saved / original_size:.1f}%), "
            f"ratio {ratio:.2f}x"
        )
        return True

    except subprocess.TimeoutExpired:
        print(f"  ERROR: timed out — original preserved.")
    except subprocess.CalledProcessError as e:
        print(f"  ERROR: ros2 bag convert failed — {e.stderr.decode().strip()}")
    except Exception as e:
        print(f"  ERROR: {e}")

    shutil.rmtree(compressed_dir, ignore_errors=True)
    return False

def main():
    log_root = os.path.expanduser("~/.ros/log")
    if len(sys.argv) > 1:
        log_root = sys.argv[1]

    if not os.path.isdir(log_root):
        print(f"Log directory not found: {log_root}")
        sys.exit(1)

    print(f"Scanning {log_root} for uncompressed bags...\n")
    bags = find_bags(log_root)

    if not bags:
        print("No bags found.")
        return

    uncompressed = [(b, is_compressed(b)) for b in bags]
    to_compress = [b for b, compressed in uncompressed if not compressed]
    already_compressed = [b for b, compressed in uncompressed if compressed]

    print(f"Found {len(bags)} bag(s): {len(to_compress)} uncompressed, {len(already_compressed)} already compressed.\n")

    if not to_compress:
        print("Nothing to do.")
        return

    ok = 0
    fail = 0
    for bag in to_compress:
        print(f"[{to_compress.index(bag)+1}/{len(to_compress)}] {bag}")
        if compress_bag(bag):
            ok += 1
        else:
            fail += 1

    print(f"\nDone. {ok} compressed, {fail} failed.")

if __name__ == "__main__":
    main()