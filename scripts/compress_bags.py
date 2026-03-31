#!/usr/bin/env python3
import os
import subprocess
import shutil
import yaml
import sys

# Colors
RESET  = '\033[0m'
BOLD   = '\033[1m'
RED    = '\033[31m'
GREEN  = '\033[32m'
YELLOW = '\033[33m'
CYAN   = '\033[36m'
DIM    = '\033[2m'

def info(msg):  print(f"  {GREEN}✔{RESET} {msg}")
def warn(msg):  print(f"  {YELLOW}⚠{RESET} {msg}")
def error(msg): print(f"  {RED}✘{RESET} {msg}")
def skip(msg):  print(f"  {DIM}⊘ {msg}{RESET}")
def step(msg):  print(f"  {CYAN}→{RESET} {msg}")

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

def is_compressed(bag_dir) -> bool:
    return bag_dir.endswith("_compressed")

def find_bags(log_root) -> list:
    bags = []
    for dirpath, dirnames, filenames in os.walk(log_root):
        if any(f.endswith(".mcap") for f in filenames):
            bags.append(dirpath)
            dirnames.clear()
    return sorted(bags)

def compress_bag(bag_dir):
    ros_log_dir = os.path.dirname(bag_dir)
    compressed_dir = bag_dir + "_compressed"
    backup_dir = bag_dir + "_backup"
    config_path = os.path.join(ros_log_dir, "compress_config.yaml")

    # Determine input path — if no metadata.yaml, pass the .mcap file directly
    has_metadata = os.path.exists(os.path.join(bag_dir, "metadata.yaml"))
    if not has_metadata:
        mcap_files = [f for f in os.listdir(bag_dir) if f.endswith(".mcap")]
        if len(mcap_files) != 1:
            skip(f"no metadata.yaml and {len(mcap_files)} .mcap files found (expected exactly 1).")
            return False
        input_path = os.path.join(bag_dir, mcap_files[0])
    else:
        input_path = bag_dir

    # Skip empty bags
    original_size = dir_size(bag_dir)
    if original_size == 0:
        skip("bag is empty (0 bytes).")
        return False

    # Skip bags with mcap files too small to be valid
    for f in os.listdir(bag_dir):
        if f.endswith(".mcap"):
            fsize = os.path.getsize(os.path.join(bag_dir, f))
            if fsize < 512:
                skip(f"{f} is too small to be a valid mcap ({fsize} bytes).")
                return False

    # Clean up leftovers from previous failed runs
    if os.path.exists(backup_dir):
        warn("Removing leftover backup from previous run...")
        shutil.rmtree(backup_dir)
    if os.path.exists(compressed_dir):
        warn("Removing leftover compressed dir from previous run...")
        shutil.rmtree(compressed_dir)

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

    step(f"Compressing {fmt_size(original_size)}...")

    try:
        subprocess.run(
            ["ros2", "bag", "convert", "--input", input_path, "mcap", "--output-options", config_path],
            check=True,
            timeout=600,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )

        if not os.path.isdir(compressed_dir) or not any(
            f.endswith(".mcap") for f in os.listdir(compressed_dir)
        ):
            error("Output bag missing or empty — original preserved.")
            shutil.rmtree(compressed_dir, ignore_errors=True)
            return False

        os.rename(bag_dir, backup_dir)
        os.rename(compressed_dir, bag_dir)
        shutil.rmtree(backup_dir)

        compressed_size = dir_size(bag_dir)
        ratio = original_size / compressed_size if compressed_size > 0 else 0
        saved = original_size - compressed_size
        info(
            f"{BOLD}{fmt_size(original_size)}{RESET} → {BOLD}{GREEN}{fmt_size(compressed_size)}{RESET}, "
            f"saved {BOLD}{fmt_size(saved)}{RESET} ({100 * saved / original_size:.1f}%), "
            f"ratio {BOLD}{ratio:.2f}x{RESET}"
        )
        return True

    except subprocess.TimeoutExpired:
        error("ros2 bag convert timed out — original preserved.")
    except subprocess.CalledProcessError as e:
        error(f"ros2 bag convert failed — {e.stderr.decode().strip()}")
    except Exception as e:
        error(f"Unexpected error: {e}")

    shutil.rmtree(compressed_dir, ignore_errors=True)
    return False

def main():
    log_root = os.path.expanduser("~/.ros/log")
    if len(sys.argv) > 1:
        log_root = sys.argv[1]

    if not os.path.isdir(log_root):
        error(f"Log directory not found: {log_root}")
        sys.exit(1)

    print(f"\n{BOLD}Scanning {CYAN}{log_root}{RESET}{BOLD} for uncompressed bags...{RESET}\n")
    bags = find_bags(log_root)

    if not bags:
        skip("No bags found.")
        return

    to_compress = [b for b in bags if not is_compressed(b)]
    already_compressed = [b for b in bags if is_compressed(b)]

    print(
        f"{BOLD}Found {len(bags)} bag(s):{RESET} "
        f"{YELLOW}{len(to_compress)} uncompressed{RESET}, "
        f"{GREEN}{len(already_compressed)} already compressed{RESET}.\n"
    )

    if not to_compress:
        info("Nothing to do.")
        return

    ok = 0
    fail = 0
    for i, bag in enumerate(to_compress):
        print(f"{BOLD}[{i+1}/{len(to_compress)}]{RESET} {DIM}{bag}{RESET}")
        if compress_bag(bag):
            ok += 1
        else:
            fail += 1

    print(f"\n{BOLD}Done.{RESET} "
          f"{GREEN}{ok} compressed{RESET}, "
          f"{RED}{fail} failed{RESET}.")

if __name__ == "__main__":
    main()