#!/bin/sh

echo "install git hooks..."

cd ../.git/hooks/

rm pre-commit
ln -sv ../../git-hooks/pre-commit.sh pre-commit

rm commit-msg
ln -sv ../../git-hooks/commit-msg.sh commit-msg
