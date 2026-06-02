#!/usr/bin/env python3
"""Recovery tool for broken Snapmaker U1 timelapse videos.

This script extracts H.264 NAL units from corrupted MP4 files and remuxes them
into a playable video file using ffmpeg.
"""
import struct
import os
import sys
import subprocess
import tempfile

# Configuration
FFMPEG_PATH = "ffmpeg"
TEMP_FILE_SUFFIX = ".temp.h264"
# Set to a directory path (e.g. '/tmp') to use a specific location.
# If None, a system temporary file will be created.
TEMP_DIR = None

# Snapshot of the headers from a known good Snapmaker U1 video file.
# SPS: 67640028acd940780227e5c05a810102a0000003002000000601e30632c0
# PPS: 68eae3cb22c0
SPS = (
    b"\x67\x64\x00\x28\xac\xd9\x40\x78\x02\x27\xe5\xc0\x5a\x81\x01\x02"
    b"\xa0\x00\x00\x03\x00\x20\x00\x00\x06\x01\xe3\x06\x32\xc0"
)
PPS = b"\x68\xea\xe3\xcb\x22\xc0"
START_CODE = b"\x00\x00\x00\x01"


def recover_file(input_path: str, output_path: str) -> bool:
    """Recover a broken Snapmaker U1 timelapse video file.

    Args:
        input_path: Path to the broken MP4 file
        output_path: Path where the recovered MP4 file will be saved

    Returns:
        True if recovery was successful, False otherwise
    """
    print(f"Recovering {input_path} -> {output_path}")

    temp_h264: str
    if TEMP_DIR:
        temp_h264 = os.path.join(
            TEMP_DIR, os.path.basename(input_path) + TEMP_FILE_SUFFIX
        )
    else:
        fd, temp_h264 = tempfile.mkstemp(suffix=TEMP_FILE_SUFFIX)
        os.close(fd)

    try:
        found_mdat = False
        with open(input_path, "rb") as f_in, open(temp_h264, "wb") as f_out:
            # 1. Write headers first (SPS/PPS)
            f_out.write(START_CODE)
            f_out.write(SPS)
            f_out.write(START_CODE)
            f_out.write(PPS)

            # 2. Find mdat atom and extract NALs
            while True:
                header = f_in.read(8)
                if len(header) < 8:
                    break

                atom_size, atom_type = struct.unpack(">I4s", header)
                atom_name = atom_type.decode("ascii", errors="ignore")

                if atom_name == "mdat":
                    print(
                        f"Found 'mdat' atom of size {atom_size} at offset "
                        f"{f_in.tell() - 8}"
                    )
                    found_mdat = True

                    # If size is 1, it's a 64-bit large atom (skip header)
                    if atom_size == 1:
                        large_size = f_in.read(8)
                        atom_size = struct.unpack(">Q", large_size)[0]
                        # We have read 8 (header) + 8 (large len) = 16 bytes.
                    else:
                        # We have read 8 bytes.
                        pass

                    # We are now at the start of NAL data inside mdat
                    # (AVCC format typically)
                    # Loop until EOF or atom end (if atom size is valid)
                    # Note: Broken files often have mdat as the last atom,
                    # sometimes with size 0 (meaning "until EOF")

                    try:
                        while True:
                            length_bytes = f_in.read(4)
                            if len(length_bytes) < 4:
                                break

                            nal_len = struct.unpack(">I", length_bytes)[0]

                            # Sanity check for NAL length
                            # (e.g., > 20MB is suspicious for this bitrate)
                            if nal_len > 20_000_000 or nal_len == 0:
                                print(
                                    f"Encountered suspicious NAL length "
                                    f"({nal_len}), stopping extraction."
                                )
                                break

                            nal_data = f_in.read(nal_len)
                            if len(nal_data) < nal_len:
                                print(
                                    "Unexpected EOF reading NAL data, "
                                    "saving what we have."
                                )
                                break

                            # Write as Annex B (Start Code + Data)
                            f_out.write(START_CODE)
                            f_out.write(nal_data)

                    except KeyboardInterrupt:
                        print("Extraction interrupted by user.")
                        raise
                    except (IOError, OSError, struct.error) as e:
                        print(f"Error during extraction: {e}")

                    # Stop looking for atoms after processing mdat
                    break

                else:
                    # Skip other atoms
                    if atom_size == 1:
                        atom_size = struct.unpack(">Q", f_in.read(8))[0]
                        f_in.seek(atom_size - 16, 1)
                    elif atom_size == 0:
                        # 0 means rest of file
                        break
                    else:
                        f_in.seek(atom_size - 8, 1)

        if not found_mdat:
            print("Error: Could not find 'mdat' atom in the input file.")
            if os.path.exists(temp_h264):
                os.remove(temp_h264)
            return False

        print("Extraction complete. Muxing to MP4...")

        # 3. Mux with ffmpeg
        ffmpeg_cmd = [
            FFMPEG_PATH,
            "-y",  # Overwrite output if exists
            "-r",
            "24",  # Force 24 fps (standard for this printer)
            "-i",
            temp_h264,
            "-c",
            "copy",  # Copy stream without re-encoding
            output_path,
        ]

        result = subprocess.run(
            ffmpeg_cmd, capture_output=True, text=True, check=False
        )

        if result.returncode != 0:
            print("FFmpeg failed to mux the video.")
            print("STDERR:", result.stderr)
            return False
        else:
            print("Recovery successful!")
            return True

    except (IOError, OSError, subprocess.SubprocessError) as e:
        print(f"An unexpected error occurred: {e}")
        return False
    finally:
        # Cleanup
        if os.path.exists(temp_h264):
            os.remove(temp_h264)
            print("Temporary files cleaned up.")


def main():
    """Main entry point for the script."""
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <broken_file.mp4> [output_file.mp4]")
        print("Recovers Snapmaker U1 timelapse videos.")
        sys.exit(1)

    in_file = sys.argv[1]

    if len(sys.argv) >= 3:
        out_file = sys.argv[2]
    else:
        base, _ = os.path.splitext(in_file)
        out_file = f"{base}_fixed.mp4"

    recover_file(in_file, out_file)


if __name__ == "__main__":
    main()
