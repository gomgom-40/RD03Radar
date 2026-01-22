#!/usr/bin/env python3
"""
RD03Radar Release Preparation Script

This script helps prepare a new release by:
1. Updating version numbers in all relevant files
2. Creating release notes from recent commits
3. Validating release readiness

Usage:
    python scripts/prepare_release.py --version 1.1.0 --dry-run
    python scripts/prepare_release.py --version 1.1.0
"""

import argparse
import re
import os
import sys
from datetime import datetime

def update_version_in_file(filepath, old_version, new_version):
    """Update version number in a file"""
    if not os.path.exists(filepath):
        print(f"⚠️  File not found: {filepath}")
        return False

    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # Replace version
    new_content = content.replace(old_version, new_version)

    if new_content != content:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(new_content)
        print(f"✅ Updated {filepath}")
        return True
    else:
        print(f"ℹ️  No changes needed in {filepath}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Prepare RD03Radar release')
    parser.add_argument('--version', required=True, help='New version number (e.g., 1.1.0)')
    parser.add_argument('--dry-run', action='store_true', help='Show what would be changed without making changes')
    parser.add_argument('--old-version', default='1.0.0', help='Old version number (default: 1.0.0)')

    args = parser.parse_args()

    print(f"🚀 Preparing RD03Radar release {args.version}")
    print("=" * 50)

    # Validate version format
    if not re.match(r'^\d+\.\d+\.\d+$', args.version):
        print(f"❌ Invalid version format: {args.version}")
        print("Version must be in format: X.Y.Z")
        sys.exit(1)

    # Files to update
    files_to_update = [
        ('RD03Radar.h', f'Version: {args.old_version}', f'Version: {args.version}'),
        ('RD03Radar.cpp', f'Version: {args.old_version}', f'Version: {args.version}'),
        ('library.properties', f'version={args.old_version}', f'version={args.version}'),
        ('README.md', f'Version-{args.old_version}-blue.svg', f'Version-{args.version}-blue.svg'),
    ]

    print("📝 Files to update:")
    for filepath, old_pattern, new_pattern in files_to_update:
        print(f"  • {filepath}: '{old_pattern}' → '{new_pattern}'")

    print("\n" + "=" * 50)

    if args.dry_run:
        print("🔍 DRY RUN - No changes will be made")
        print("=" * 50)
    else:
        print("⚡ MAKING CHANGES")
        print("=" * 50)

    changes_made = 0

    for filepath, old_pattern, new_pattern in files_to_update:
        if args.dry_run:
            print(f"Would update: {filepath}")
            print(f"  '{old_pattern}' → '{new_pattern}'")
        else:
            if update_version_in_file(filepath, old_pattern, new_pattern):
                changes_made += 1

    print("\n" + "=" * 50)

    if not args.dry_run:
        if changes_made > 0:
            print(f"✅ Successfully updated {changes_made} files")

            # Suggest next steps
            print("\n📋 Next steps:")
            print("1. Review and commit changes:")
            print(f"   git add . && git commit -m 'Release v{args.version}'")
            print("2. Create and push tag:")
            print(f"   git tag v{args.version} && git push origin v{args.version}")
            print("3. Create GitHub release:")
            print("   Go to GitHub → Releases → Create new release"
            print("4. Wait for CI/CD to complete and Arduino Library Manager update"
        else:
            print("ℹ️  No files were updated")

    # Generate release notes suggestion
    print("
📝 Suggested Release Notes:"    print(f"""
# RD03Radar v{args.version} - [Brief Description]

## What's New
- [List major new features]

## Enhancements
- [List improvements and bug fixes]

## Compatibility
- ESP32 & ESP8266 support
- Arduino IDE 1.8.x+
- PlatformIO compatible

## Installation
Download ZIP or install via Arduino Library Manager.

---
Made with ❤️ by Mohamed Eid (gomgom-40)
""")

if __name__ == "__main__":
    main()
