#!/bin/bash
#

# This way you can customize which branches should be skipped when
# prepending commit message.

COMMIT_MSG_HEADER=(Add Fix Remove Change Merge Update)

#COMMIT_MSG_HEADER_EXIST=$(grep -c "

BRANCHES_TO_SKIP=(master develop)

BRANCH_NAME=$(git symbolic-ref --short HEAD)
BRANCH_NAME="${BRANCH_NAME##*/}"

BRANCH_EXCLUDED=$(printf "%s\n" "${BRANCHES_TO_SKIP[@]}" | grep -c "^$BRANCH_NAME$")
BRANCH_IN_COMMIT=$(grep -c "\[$BRANCH_NAME\]" $1)

if [[ -n "$BRANCH_NAME" ]] && ! [[ $BRANCH_EXCLUDED -eq 1 ]] && ! [[ $BRANCH_IN_COMMIT -ge 1 ]]; then
  sed -i.bak -e "1s/^/[$BRANCH_NAME] /" $1
fi

echo "" >> "$1"
echo "####################" >> "$1"
echo "Code Static Check Passed (cpplint.py)" >> "$1"
