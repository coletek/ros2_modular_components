import time
import mysql.connector

class DBSingleton:
    _instance = None
    _mydb = None
    _mycursor = None
    _is_debug = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(DBSingleton, cls).__new__(cls)
        return cls._instance

    def is_debug(self, is_debug):
        self._is_debug = is_debug
    
    def setup(self, db_host, db_name, db_username, db_password):
        if self._is_debug:
            print ("%s DB::setup(%s, %s, %s, %s)" % (time.time(), db_host, db_name, db_username, db_password))
        self._mydb = mysql.connector.connect(
            host=db_host,
            user=db_username,
            passwd=db_password,
            database=db_name
        )
        self._mycursor = self._mydb.cursor(dictionary=True, buffered = True)

    def query(self, query):
        #if self._is_debug:
        #  print ("%s DB::query(%s)" % (time.time(), query))
        self._mycursor.execute(query)

    def get(self, query, is_one_entry = True):
        self._mycursor.execute(query)
        if is_one_entry:
            data = self._mycursor.fetchone()
        else:
            data = self._mycursor.fetchall()
        #if self._is_debug:
        #if data:
        #  print ("%s DB::get(%s, %r) data(%d)=" % (time.time(), query, is_one_entry, len(data)))
        #  print (data)
        #else:
        #  print ("%s DB::get(%s, %r) data(0)=None" % (time.time(), query, is_one_entry))

        if is_one_entry and data and len(data) == 1:
            k, v = data.popitem()
            return v
    
        return data

    def insert(self, tablename, data):
        timestamp = time.time()

        data['added_date'] = timestamp
        data['updated_date'] = timestamp
        
        cols = []
        vals = []
        ss = []
        
        for k, v in data.items():
            cols.append(k)
            vals.append(v)
            ss.append("%s")
      
        query = "INSERT INTO %s (%s) VALUES (%s)" % (tablename, ", ".join(cols), ", ".join(ss))
        #if self._is_debug:
        #  print ("%s DB::insert(%s, '%s') query=%s" % (timestamp, tablename, ", ".join(data), query))
        self._mycursor.execute(query, vals)
        self._mydb.commit()

        query = "SELECT LAST_INSERT_ID()"
        return self.get(query)
  
    def update(self, tablename, data, where):
        query = "SELECT id FROM %s %s" % (tablename, where)
        if not self.get(query):
            return self.insert(tablename, data)
        timestamp = time.time()
        #if self._is_debug:
        #  print ("%s DB::update(%s, %s, %s)" % (timestamp, tablename, ", ".join(data), where))
        data_list = []
        for k, v in data.items():
            if v:
                data_list.append("%s = '%s'" % (k, v))
        data_list.append("updated_date = %f" % timestamp)
        query = "UPDATE %s SET %s %s" % (tablename, ", ".join(data_list), where)
        self.query(query)
