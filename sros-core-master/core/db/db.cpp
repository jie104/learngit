//
// Created by john on 19-10-25.
//

#include "db.h"

std::recursive_mutex g_db_mutex;

SQLite::Database g_db(db::DB_PATH, db::OPEN_FLAGS, db::BUSY_TIMEOUT);