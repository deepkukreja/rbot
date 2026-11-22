#!/usr/bin/env python3
"""Verify pinned upstream releases and materialize the optional source overlay."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import tarfile
import tempfile
from urllib.request import urlopen


HERE = Path(__file__).resolve().parent


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def archive_for(entry, cache):
    path = cache / entry['archive']
    if path.is_file():
        if digest(path) != entry['sha256']:
            raise ValueError(f'Archive SHA256 mismatch: {path}')
        return path
    with tempfile.NamedTemporaryFile(dir=cache, prefix='.download-') as staging:
        with urlopen(entry['url'], timeout=120) as response:
            shutil.copyfileobj(response, staging)
        staging.flush()
        if digest(Path(staging.name)) != entry['sha256']:
            raise ValueError(f'Upstream archive SHA256 mismatch: {entry["archive"]}')
        shutil.copy2(staging.name, path)
    return path


def unpack(archive, entry, destination):
    prefix = entry['prefix'] + '/'
    with tarfile.open(archive, 'r:gz') as release:
        for member in release:
            if not member.isfile() or not member.name.startswith(prefix):
                continue
            relative = PurePosixPath(member.name[len(prefix):])
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError(f'Unexpected archive path: {member.name}')
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            with release.extractfile(member) as original, target.open('wb') as output:
                shutil.copyfileobj(original, output)
        license_path = destination / 'LICENSE'
        with release.extractfile(prefix.split('/')[0] + '/LICENSE') as license_file:
            license_path.write_bytes(license_file.read())
    subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p1',
                    '-i', str(HERE / entry['patch'])], cwd=destination, check=True)
    for name, fingerprint in entry['patched_sha256'].items():
        if digest(destination / name) != fingerprint:
            raise ValueError(f'Patched file SHA256 mismatch: {name}')


def prepare(destination, cache):
    if destination.exists():
        raise ValueError(f'Choose a new source directory: {destination}')
    cache.mkdir(parents=True, exist_ok=True)
    destination.parent.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((HERE / 'sources.json').read_text())
    with tempfile.TemporaryDirectory(prefix='.coupled-source-', dir=destination.parent) as staging:
        source = Path(staging) / 'src'
        source.mkdir()
        for package, entry in manifest.items():
            unpack(archive_for(entry, cache), entry, source / package)
        shutil.copytree(HERE / 'rlai_drive_constraints', source / 'rlai_drive_constraints')
        source.rename(destination)
    print(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--archive-cache', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.destination.resolve(), args.archive_cache.resolve())
