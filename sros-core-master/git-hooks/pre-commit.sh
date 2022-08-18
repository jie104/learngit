#!/bin/bash
#
# Modified from http://qiita.com/janus_wel/items/cfc6914d6b7b8bf185b6
#
# An example hook script to verify what is about to be committed.
# Called by "git commit" with no arguments.  The hook should
# exit with non-zero status after issuing an appropriate message if
# it wants to stop the commit.
#
# To enable this hook, rename this file to "pre-commit".

CPPLINT_LEVEL=2
CPPLINT_FILTER=-readability/fn_size,-build/namespaces,-build/include_subdir,-build/include,-runtime/string,-runtime/references,-runtime/printf
CPPLINT_LINELENGTH=120

if git rev-parse --verify HEAD >/dev/null 2>&1
then
    against=HEAD
else
    # Initial commit: diff against an empty tree object
    against=4b825dc642cb6eb9a060e54bf8d69288fbee4904
fi

# Redirect output to stderr.
exec 1>&2

project_dir=$(cd $(dirname $0); cd $(dirname $(ls -l $(basename $0) | awk '{print $NF}')); pwd)/.. # sros-core dir

CPPLINT="${project_dir}/cpplint.py --filter=${CPPLINT_FILTER} --linelength=${CPPLINT_LINELENGTH}"
sum=0

# for cpp
for file in $(git diff --cached --name-status | grep -E '\.[ch](pp)?$' | awk '{print $2}'); do
    $CPPLINT ${project_dir}/$file
    sum=$(expr ${sum} + $?)
done

if [[ ${sum} -eq 0 ]]; then
    exit 0
else
    exit 1
fi

