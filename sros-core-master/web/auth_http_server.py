import os
import base64
import SocketServer
import BaseHTTPServer
import SimpleHTTPServer

from db_util import *


class AuthHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    ''' Main class to present webpages and authentication. '''
    def do_AUTHHEAD(self):
        self.send_response(401)
        self.send_header('WWW-Authenticate', 'Basic realm=\"Auth\"')
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        global keys
        ''' Present frontpage with user authentication. '''
        if self.headers.getheader('Authorization') == None:
            self.do_AUTHHEAD()
            self.wfile.write('No auth header received')
        else:
            for key in keys:
                if self.headers.getheader('Authorization') == 'Basic ' + key:
                    SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)
                    return
            else:
                self.do_AUTHHEAD()
                self.wfile.write(self.headers.getheader('Authorization'))
                self.wfile.write('Not authenticated')


class ThreadingSimpleServer(SocketServer.ThreadingMixIn,
                            BaseHTTPServer.HTTPServer):
    pass


if __name__ == '__main__':

    print "load configuration from db"

    db = DBUtil()
    port = db.get_config_value("web.auth_http_server_port")
    username = db.get_config_value("web.auth_http_server_username")
    passwd = db.get_config_value("web.auth_http_server_passwd")

    if port == "":
        port = 8000

    keys = map(base64.b64encode, ["%s:%s" % (username, passwd)])

    server = ThreadingSimpleServer(('', int(port)), AuthHandler)
    try:
        print "Start server at port", port
        server.serve_forever()
    except KeyboardInterrupt:
        print('Finished')
