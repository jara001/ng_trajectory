#!/bin/sh
# release.sh
# Release a new version.

# Check whether we have an argument.
if test $# -ne 1; then
    echo -e "Usage: sh release.sh VERSION"
    exit 1;
fi


# 1) Create new branch
git checkout -b release/$1


# 2) Update CHANGELOG
sed -z "s/## Unreleased/## Unreleased\n## $1 - `date +%Y-%m-%d`/" -i CHANGELOG.md


# 3) Re-generate README
mkdir -p doc
./bin/ng_help --gendoc > doc/README.md


# 3b) Update citation
sed -i "s/^version: .*/version: $1/g" CITATION.cff
sed -i "s/^date-released: .*/date-released: `date +%Y-%m-%d`/g" CITATION.cff


# 4) Add changes
git add CHANGELOG.md README.md CITATION.cff doc/README.md


# 5) Commit changes
git commit -m "Release version $1"


# 6) Move to master
git checkout master


# 7) Merge branches
git merge release/$1 --no-ff --no-edit


# 8) Get tag message
LINESTART=$(cat CHANGELOG.md | grep -n "^## " | head -n 2 | tail -n 1 | cut -d: -f1)
LINEEND=$(cat CHANGELOG.md | grep -n "^## " | head -n 3 | tail -n 1 | cut -d: -f1)
MESSAGE=$(head -n $LINEEND CHANGELOG.md | tail -n +$LINESTART | head -n -1 | tail -n +2)


# 9) Tag commit
git tag -a "v$1" --cleanup=whitespace -m "$MESSAGE"


# 10) Generate new wheel
make build
