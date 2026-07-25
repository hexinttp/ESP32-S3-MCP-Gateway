import gzip
import pathlib
import sys


def main() -> None:
    source = pathlib.Path(sys.argv[1])
    destination = pathlib.Path(sys.argv[2])
    compressed = gzip.compress(source.read_bytes(), compresslevel=9, mtime=0)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(compressed)


if __name__ == "__main__":
    main()
