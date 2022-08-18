#!/bin/bash

# check every commit msg, if no $CHECK_PASSED in commit msg, declined push request

CHECK_PASSED="Code Static Check Passed"

while read OLDREV NEWREV REFNAME ; do
  for COMMIT in `git rev-list $OLDREV..$NEWREV`;
  do
    HAS_PASSED=`git cat-file commit $COMMIT | grep -c "${CHECK_PASSED}"`
    if ! [[ $HAS_PASSED -eq 1 ]]; then
      echo -e "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
      echo -e "+                                                        +"
      echo -e "+   [REJECT] THE COMMIT DIDN'T PASS CODE STATIC CHECK.   +"
      echo -e "+                                                        +"
      echo -e "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
      exit 1
    fi
  done
done

exit 0
