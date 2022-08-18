import sqlite3
import traceback
import time

SROS_DB_PATH = "/sros/db/main.db3"


class DBUtil(object):
    def __init__(self, db_path=SROS_DB_PATH):
        self.db = sqlite3.connect(db_path)
        self.cur = self.db.cursor()
        self.retry = 3

    def get_config_value(self, key):
        self.retry = 3

        while self.retry > 0:
            try:
                self.cur.execute("SELECT value FROM config where key = '%s' and is_valid=1;" % key)
                rs = self.cur.fetchone()
                if rs is not None and len(rs) != 0:
                    return rs[0].encode('ascii', 'ignore')
                else:
                    return ""
            except:
                traceback.print_exc()
                self.retry = self.retry - 1
                time.sleep(0.1)

        return ""
