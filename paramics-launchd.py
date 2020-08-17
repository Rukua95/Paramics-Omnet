#!/usr/bin/env python
# coding=utf-8

# paramics-launchd.py -- Modified version of sumo-launchd.py for PVeins,
# the Paramics-VEINS plugin.
# Copyright (C) 2017- Manuel Olguín <molguin@dcc,uchile.cl>
#
# sumo-launchd.py -- SUMO launcher daemon for use with TraCI clients
# Copyright (C) 2006-2012 Christoph Sommer <christoph.sommer@uibk.ac.at>
#
# Documentation for these modules is at http://veins.car2x.org/
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#

"""
For each incoming TCP connection the daemon receives a launch configuration.
It starts SUMO accordingly, then proxies all TraCI Messages.
The launch configuration must be sent in the very first TraCI message.
This message must contain a single command, CMD_FILE_SEND and be used to
send a file named "paramics-launchd.launch.xml", which has the following
structure:
<?xml version="1.0"?>
<!-- debug config -->
<launch>
    <basedir path="C:\Users\Public\paramics\programmer\plugins\pveins" />
	<network name="example_network" />
    <seed value="1234" />
</launch>
"""

import os
import stat
import sys
import tempfile
import shutil
import socket
import struct
import subprocess
import time
import signal
import exceptions
import thread
import xml.dom.minidom
import select
import logging
import atexit
import fileinput
import zipfile
import random
from optparse import OptionParser

_API_VERSION = 1
_LAUNCHD_VERSION = 'paramics-launchd.py 1.00'
_CMD_GET_VERSION = 0x00
_CMD_FILE_SEND = 0x75
_SIMULATION_MODE = 'modeller'
#_SIMULATION_MODE = 'processor'


class UnusedPortLock:
    lock = thread.allocate_lock()

    def __init__(self):
        self.acquired = False

    def __enter__(self):
        self.acquire()

    def __exit__(self):
        self.release()

    def acquire(self):
        if not self.acquired:
            logging.debug("Claiming lock on port")
            UnusedPortLock.lock.acquire()
            self.acquired = True

    def release(self):
        if self.acquired:
            logging.debug("Releasing lock on port")
            UnusedPortLock.lock.release()
            self.acquired = False


def find_unused_port():
    """
    Return an unused port number.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
    sock.bind(('127.0.0.1', 0))
    sock.listen(socket.SOMAXCONN)
    ipaddr, port = sock.getsockname()
    sock.close()
    return port


def forward_connection(client_socket, server_socket, process):
    """
    Proxy connections until either socket runs out of data or process terminates.
    """

    logging.debug("Starting proxy mode")

    client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    do_exit = False
    while not do_exit:

        data = ""
        
        (r, w, e) = select.select(
            [client_socket, server_socket], [], [client_socket, server_socket], 1)
        if client_socket in e:
            logging.debug("error in client socket??")
            do_exit = True
            
        if server_socket in e:
            logging.debug("error in server socket??")
            do_exit = True
        if client_socket in r:
            try:
                #logging.debug("reading data from client")
                data = client_socket.recv(65535)
                s_data = ':'.join(x.encode('hex') for x in data)
                #logging.debug("Data: " + s_data)
                if data == "":
                    do_exit = True
            except Exception as e:
                logging.debug(str(e))
                logging.debug("Error when receiving from client?")
                do_exit = True
            finally:
                #logging.debug("sending data to server")
                server_socket.send(data)
        if server_socket in r:
            try:
                #logging.debug("reading data from server")
                data = server_socket.recv(65535)
                s_data = ':'.join(x.encode('hex') for x in data)
                #logging.debug("Data: " + s_data)
                if data == "":
                    do_exit = True
            except Exception as e:
                logging.debug(str(e))
                logging.debug("Error when receiving from server?")
                do_exit = True
            finally:
                #logging.debug("sending data to client")
                client_socket.send(data)

    logging.debug("Done with proxy mode")


def parse_launch_configuration(launch_xml_string):
    """
    Returns tuple of options set in launch configuration
    """

    p = xml.dom.minidom.parseString(launch_xml_string)

    # get root node "launch"
    launch_node = p.documentElement
    if (launch_node.tagName != "launch"):
        raise RuntimeError(
            "launch config root element not <launch>, but <%s>" % launch_node.tagName)

    # get "launch.basedir"
    basedir = ""
    basedir_nodes = [x for x in launch_node.getElementsByTagName(
        "basedir") if x.parentNode == launch_node]
    if len(basedir_nodes) > 1:
        raise RuntimeError(
            'launch config contains %d <basedir> nodes, expected at most 1' % (len(basedir_nodes)))
    elif len(basedir_nodes) == 1:
        basedir = basedir_nodes[0].getAttribute("path")
    logging.debug("Base dir is %s" % basedir)

    # get "launch.seed"
    seed = 0
    logging.debug("Initial seed value is %d" % seed)
    seed_nodes = [x for x in launch_node.getElementsByTagName("seed") if x.parentNode == launch_node]
    if len(seed_nodes) > 1:
        raise RuntimeError(
            'launch config contains %d <seed> nodes, expected at most 1' % (len(seed_nodes)))
    elif len(seed_nodes) == 1:
        seed = int(seed_nodes[0].getAttribute("value"))

    if seed == 0:
        logging.debug("Generating random seed")
        seed = random.randint(0, 999999)
    logging.debug("Seed is %d" % seed)

    # get "launch.network"
    network = ""
    network_nodes = [x for x in launch_node.getElementsByTagName("network") if x.parentNode == launch_node]
    if len (network_nodes) != 1:
        raise RuntimeError(
            'launch config contains %d <network> nodes, expected exactly 1' % (len(network_nodes)))
    else:
        network = network_nodes[0].getAttribute("name")
    logging.debug("Network is %s" % network)
    

    return (basedir, network, seed)


def run_paramics(command, network, runpath, shlex, remote_port, seed, client_socket, unused_port_lock):
    """
    Actually run Paramics.
    :param network: 
    """

    # create log files
    paramicsLogOut = open(os.path.join(runpath, 'paramics-launchd.out.log'), 'w')
    paramicsLogErr = open(os.path.join(runpath, 'paramics-launchd.err.log'), 'w')

    # start paramics
    paramics_start = int(time.time())
    paramics_end = None
    paramics_returncode = -1
    paramics_status = None
    try:
        cmd = []
        if shlex:
            import shlex
            cmd = shlex.split(command.replace(
                '{}', + unicode(runpath).encode()))
        else:
            cmd = [command, network]#, "--traci_port={}".format(remote_port)]

        logging.info("Starting paramics (%s) on port %d, seed %d" %
                     (" ".join(cmd), remote_port, seed))
        paramics = subprocess.Popen(
            cmd, cwd=runpath, stdin=None, stdout=paramicsLogOut, stderr=paramicsLogErr)

        paramics_socket = None

        connected = False
        tries = 1
        while not connected:
            try:
                logging.debug("Connecting to paramics (%s) on port %d (try %d)" % (
                    " ".join(cmd), remote_port, tries))
                paramics_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                paramics_socket.connect(('127.0.0.1', remote_port))
                break

            except socket.error, e:
                logging.debug("Error (%s)" % e)
                if tries >= 10:
                    raise
                time.sleep(tries * 0.25)
                tries += 1

        unused_port_lock.release()
        forward_connection(client_socket, paramics_socket, paramics)

        client_socket.close()
        paramics_socket.close()

        logging.debug("Done with proxy mode, killing paramics")

        thread.start_new_thread(subprocess.Popen.wait, (paramics,))
        time.sleep(0.5)
        if paramics.returncode == None:
            logging.debug("SIGTERM")
            os.kill(paramics.pid, signal.SIGTERM)
            time.sleep(0.5)
            if paramics.returncode == None:
                logging.debug("SIGKILL")
                os.kill(paramics.pid, signal.SIGKILL)
                time.sleep(1)
                if paramics.returncode == None:
                    logging.debug(
                        "Warning: paramics still not dead. Waiting 10 more seconds...")
                    time.sleep(10)

        logging.info("Done running paramics")
        paramics_returncode = paramics.returncode
        if paramics_returncode == 0:
            paramics_status = "Done."
        elif paramics_returncode != None:
            paramics_status = "Exited with error code %d" % paramics_returncode
        else:
            paramics_returncode = -1
            paramics_status = "Undef"

    except OSError, e:
        paramics_status = "Could not start paramics (%s): %s" % (" ".join(cmd), e)

    except exceptions.SystemExit:
        paramics_status = "Premature launch script exit"

    except exceptions.KeyboardInterrupt:
        paramics_status = "Keyboard interrupt."

    except socket.error, e:
        paramics_status = "Could not connect to paramics (%s). Might be protected by a personal firewall or crashed before a connection could be established." % e

    except:
        raise

    # statistics
    paramics_end = int(time.time())

    # close log files
    paramicsLogOut.close()
    paramicsLogErr.close()

    # read log files
    paramicsLogOut = open(os.path.join(runpath, 'paramics-launchd.out.log'), 'r')
    paramicsLogErr = open(os.path.join(runpath, 'paramics-launchd.err.log'), 'r')
    paramics_stdout = paramicsLogOut.read()
    paramics_stderr = paramicsLogErr.read()
    paramicsLogOut.close()
    paramicsLogErr.close()

    # prepare result XML
    CDATA_START = '<![CDATA['
    CDATA_END = ']]>'
    result_xml = '<?xml version="1.0"?>\n'
    result_xml += '<status>\n'
    result_xml += '\t<%s>%s</%s>\n' % ("exit-code",
                                       paramics_returncode, "exit-code")
    if paramics_start:
        result_xml += '\t<%s>%s</%s>\n' % ("start", paramics_start, "start")
    if paramics_end:
        result_xml += '\t<%s>%s</%s>\n' % ("end", paramics_end, "end")
    if paramics_status:
        result_xml += '\t<%s>%s</%s>\n' % ("status", paramics_status, "status")
    result_xml += '\t<%s>%s</%s>\n' % ("stdout", CDATA_START + paramics_stdout.replace(
        CDATA_END, CDATA_END + CDATA_END + CDATA_START) + CDATA_END, "stdout")
    result_xml += '\t<%s>%s</%s>\n' % ("stderr", CDATA_START + paramics_stderr.replace(
        CDATA_END, CDATA_END + CDATA_END + CDATA_START) + CDATA_END, "stderr")
    result_xml += '</status>\n'

    return result_xml


def copy_and_modify_files(basedir, network_name, runpath, seed, plugin, remote_port):
    """
    Copy (and modify) files
    :param network_name: 
    """
    logging.debug("Copying and modifying files for use.")
    orig_network_dir = os.path.join(basedir, network_name)
    new_network_dir = os.path.join(runpath, network_name)

    # create a zip with the network
    zip_name = shutil.make_archive(orig_network_dir, 'zip', basedir, network_name, 1)
    zip_path = os.path.join(basedir, zip_name)
    new_zip_path = os.path.join(runpath, zip_name)

    logging.debug("Original network path: %s" % orig_network_dir)
    logging.debug("Network zip path: %s" % zip_path)

    shutil.move(zip_path, new_zip_path)
    zip_ref = zipfile.ZipFile(new_zip_path, 'r')
    zip_ref.extractall(runpath)
    zip_ref.close()

    # delete zip files
    #os.remove(zip_path)
    os.remove(new_zip_path)

    logging.debug("Copied all files. Modifying copies for VEINS use.")

    # modify config to include seed
    new_config_file = os.path.join(new_network_dir, "configuration")
    port_file = os.path.join(new_network_dir, "port")
    plugin_file = ""

	# plugin folder for modeller
    if _SIMULATION_MODE == 'modeller':
        plugin_file = os.path.join(new_network_dir, "programming.modeller")

	# plugin folder for processor
    elif _SIMULATION_MODE == 'processor':
	    plugin_file = os.path.join(new_network_dir, "programming")

    else:
	    raise RuntimeError("Simulation mode doesn't existe")

    seed_config = "seed {}".format(seed)


    print "\nplugin_file = ", plugin_file
    print "plugin = ", plugin
    print "orig_network_dir = ", orig_network_dir
    print "port_file = ", port_file
    print "remote_port = ", remote_port
    print "new_network_dir = ", new_network_dir, "\n"


    if not os.path.exists(new_config_file):
        # config file doesn't exist, create it and add seed line
        with open(new_config_file, 'w') as f:
            f.write(seed_config)
    else:
        # replace seed value inplace using fileinput
        replaced = False
        for line in fileinput.input(new_config_file, inplace=True):
            if line.startswith("seed") and not replaced:
                print seed_config
                replaced = True
            else:
                print line

        fileinput.close()

        # check that the config was replaced, and if not, add it
        if not replaced:
            with open(new_config_file, 'a') as f:
                f.write(seed_config)

    # finally, add plugin file
    if not os.path.exists(port_file):
        with open(port_file, 'w') as f:
            f.write(str(remote_port))
    else:
        with open(port_file, 'a') as f:
            f.write(str(remote_port))

    # finally, add plugin file
    if not os.path.exists(plugin_file):
        with open(plugin_file, 'w') as f:
            f.write(plugin)
    else:
        with open(plugin_file, 'a') as f:
            f.write(plugin)


def handle_launch_configuration(command, shlex, launch_xml_string, client_socket, keep_temp, plugin):
    """
    Process launch configuration in launch_xml_string.
    :param plugin: 
    """

    # create temporary directory
    logging.debug("Creating temporary directory...")
    runpath = tempfile.mkdtemp(prefix="paramics-launchd-tmp-")
    if not runpath:
        raise RuntimeError("Could not create temporary directory")
    if not os.path.exists(runpath):
        raise RuntimeError(
            'Temporary directory "%s" does not exist, even though it should have been created' % runpath)
    logging.debug("Temporary dir is %s" % runpath)

    result_xml = None
    unused_port_lock = UnusedPortLock()
    try:
        # parse launch configuration
        (basedir, network, seed) = parse_launch_configuration(launch_xml_string)

        # find remote_port
        logging.debug("Finding free port number...")
        unused_port_lock.__enter__()
        remote_port = find_unused_port()
        logging.debug("...found port %d" % remote_port)

        # copy (and modify) files
        copy_and_modify_files(basedir, network, runpath, seed, plugin, remote_port)

        # run Paramics
        result_xml = run_paramics(command, network, runpath, shlex, remote_port, seed, client_socket, unused_port_lock)
    finally:
        unused_port_lock.__exit__()

        # clean up
        if not keep_temp:
            logging.debug("Cleaning up")
            shutil.rmtree(runpath)
        else:
            logging.debug("Not cleaning up %s" % runpath)

        logging.debug('Result: \n"%s"\n\n' % result_xml)

    return result_xml


def handle_get_version(conn):
    """
    process a "get version" command received on the connection
    """

    logging.debug('Got CMD_GETVERSION')

    # Send OK response and version info
    response = struct.pack("!iBBBiBBii", 4 + 1 + 1 + 1 + 4 + 1 + 1 + 4 + 4 + len(_LAUNCHD_VERSION), 1 + 1 + 1 + 4,
                           _CMD_GET_VERSION,
                           0x00, 0x00, 1 + 1 + 4 + 4 + len(_LAUNCHD_VERSION), _CMD_GET_VERSION, _API_VERSION,
                           len(_LAUNCHD_VERSION)) + _LAUNCHD_VERSION
    conn.send(response)


def read_launch_config(conn):
    """
    Read (and return) launch configuration from socket
    """

    # Get TraCI message length
    msg_len_buf = ""
    while len(msg_len_buf) < 4:
        msg_len_buf += conn.recv(4 - len(msg_len_buf))
    msg_len = struct.unpack("!i", msg_len_buf)[0] - 4

    logging.debug("Got TraCI message of length %d" % msg_len)

    # Get TraCI command length
    cmd_len_buf = ""
    cmd_len_buf += conn.recv(1)
    cmd_len = struct.unpack("!B", cmd_len_buf)[0] - 1
    if cmd_len == -1:
        cmd_len_buf = ""
        while len(cmd_len_buf) < 4:
            cmd_len_buf += conn.recv(4 - len(cmd_len_buf))
        cmd_len = struct.unpack("!i", cmd_len_buf)[0] - 5

    logging.debug("Got TraCI command of length %d" % cmd_len)

    # Get TraCI command ID
    cmd_id_buf = ""
    cmd_id_buf += conn.recv(1)
    cmd_id = struct.unpack("!B", cmd_id_buf)[0]

    logging.debug("Got TraCI command 0x%x" % cmd_id)

    if cmd_id == _CMD_GET_VERSION:
        # handle get version command
        handle_get_version(conn)
        # ...and try reading the launch config again
        return read_launch_config(conn)
    elif cmd_id != _CMD_FILE_SEND:
        raise RuntimeError(
            "Expected CMD_FILE_SEND (0x%x), but got 0x%x" % (_CMD_FILE_SEND, cmd_id))

    # Get File name
    fname_len_buf = ""
    while len(fname_len_buf) < 4:
        fname_len_buf += conn.recv(4 - len(fname_len_buf))
    fname_len = struct.unpack("!i", fname_len_buf)[0]
    fname = conn.recv(fname_len)

    # note: VEINS automatically renames launch config to
    # "sumo-launchd.launch.xml", so we have to keep it this way
    if fname != "sumo-launchd.launch.xml":
        raise RuntimeError(
            'Launch configuration must be named "sumo-launchd.launch.xml", got "%s" instead.' % fname)

    logging.debug('Got CMD_FILE_SEND for "%s"' % fname)

    # Get File contents
    data_len_buf = ""
    while len(data_len_buf) < 4:
        data_len_buf += conn.recv(4 - len(data_len_buf))
    data_len = struct.unpack("!i", data_len_buf)[0]
    data = conn.recv(data_len)

    logging.debug('Got CMD_FILE_SEND with data \n"%s"' % data)

    # Send OK response
    response = struct.pack("!iBBBi", 4 + 1 + 1 + 1 + 4,
                           1 + 1 + 1 + 4, _CMD_FILE_SEND, 0x00, 0x00)
    conn.send(response)

    return data


def handle_connection(command, shlex, conn, addr, keep_temp, plugin):
    """
    Handle incoming connection.
    :param plugin: 
    """

    logging.debug("Handling connection from %s on port %d" % addr)

    try:
        data = read_launch_config(conn)
        handle_launch_configuration(command, shlex, data, conn, keep_temp, plugin)

    except Exception, e:
        logging.error("Aborting on error: %s" % e)

    finally:
        logging.debug("Closing connection from %s on port %d" % addr)
        conn.close()


def wait_for_connections(command, shlex, port, bind_address, do_daemonize, do_kill, pidfile, keep_temp, plugin):
    """
    Open TCP socket, wait for connections, call handle_connection for each
    :param plugin: 
    """

    if do_kill:
        check_kill_daemon(pidfile)

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((bind_address, port))
    listener.listen(5)
    logging.info("Listening on port %d" % port)

    if do_daemonize:
        logging.info("Detaching to run as daemon")
        daemonize(pidfile)

    try:
        while True:
            conn, addr = listener.accept()
            logging.debug("Connection from %s on port %d" % (addr, port))
            thread.start_new_thread(
                handle_connection, (command, shlex, conn, addr, keep_temp, plugin))

    except exceptions.SystemExit:
        logging.warning("Killed.")

    except exceptions.KeyboardInterrupt:
        logging.warning("Keyboard interrupt.")

    except:
        raise

    finally:
        # clean up
        logging.info("Shutting down.")
        listener.close()


def check_kill_daemon(pidfile):
    # check pidfile, see if the daemon is still running
    try:
        pidfileh = open(pidfile, 'r')
        old_pid = int(pidfileh.readline())
        if old_pid:
            logging.info(
                "There might already be a daemon running with PID %d. Sending SIGTERM." % old_pid)
            try:
                os.kill(old_pid, signal.SIGTERM)
                time.sleep(1)
            except OSError, e:
                pass

        pidfileh.close()
    except IOError, e:
        pass


def daemonize(pidfile):
    """
    detach process, keep it running in the background
    """

    # fork and exit parent process
    try:
        child_pid = os.fork()
        if child_pid > 0:
            # parent can exit
            sys.exit(0)
        elif child_pid == 0:
            # child does nothing
            pass
        else:
            # logging.error("Aborting. Failed to fork: %s" % e.strerror)
            sys.exit(1)
    except OSError, e:
        logging.error("Aborting. Failed to fork: %s" % e.strerror)
        sys.exit(1)

    # get rid of any outside influence
    os.setsid()

    # fork again to prevent zombies
    try:
        child_pid = os.fork()
        if child_pid > 0:
            # parent can exit
            sys.exit(0)
        elif child_pid == 0:
            # child creates PIDFILE
            logging.info("Fork successful. PID is %d" % os.getpid())
            if pidfile:
                pidfileh = open(pidfile, 'w')
                pidfileh.write('%d\n' % os.getpid())
                pidfileh.close()
                atexit.register(os.remove, pidfile)
        else:
            # logging.error("Aborting. Failed to fork: %s" % e.strerror)
            sys.exit(1)

    except OSError, e:
        logging.error("Aborting. Failed to fork: %s" % e.strerror)
        sys.exit(1)


def main():
    """
    Program entry point when run interactively.
    """

    #################################################
    # Path hacia ejecutables de paramics
    command = 'C:\\Program Files (x86)\\paramicsv6\\'
    if _SIMULATION_MODE == 'modeller':
        command = command + "Modeller.exe"
    elif _SIMULATION_MODE == 'processor':
	    command = command + "processor-cmd.exe"
    else:
		raise RuntimeError("Simulation mode doesn't exist")
    #################################################

    # Option handling
    parser = OptionParser()
    parser.add_option("-c", "--command", dest="command", default=os.path.expandvars(command),
                      help="run Paramics as COMMAND [default: %default]. By default, runs the Paramics Modeller module.",
                      metavar="COMMAND")

    #################################################
    # Path hacia plugin
    parser.add_option("-u", "--plugin", dest="plugin", default=os.path.join("C:\Users\\thi-s\Documents\NicLabs\Paramics-Omnet\Omnet-Veins", "modeller.dll"),
                      help="location of the TraCI Paramics Plugin [default: %plugin]", metavar="PLUGIN")
    #################################################

    parser.add_option("-s", "--shlex", dest="shlex", default=False, action="store_true",
                      help="treat command as shell string to execute, replace {} with command line parameters [default: no]")
    parser.add_option("-p", "--port", dest="port", type="int", default=9999, action="store",
                      help="listen for connections on PORT [default: %default]", metavar="PORT")
    parser.add_option("-b", "--bind", dest="bind", default="127.0.0.1",
                      help="bind to ADDRESS [default: %default]", metavar="ADDRESS")
    parser.add_option("-L", "--logfile", dest="logfile", default=os.path.join(tempfile.gettempdir(),
                                                                              "paramics-launchd.log"),
                      help="log messages to LOGFILE [default: %default]", metavar="LOGFILE")
    parser.add_option("-v", "--verbose", dest="count_verbose", default=0,
                      action="count", help="increase verbosity [default: don't log infos, debug]")
    parser.add_option("-q", "--quiet", dest="count_quiet", default=0,
                      action="count", help="decrease verbosity [default: log warnings, errors]")
    parser.add_option("-d", "--daemon", dest="daemonize", default=False,
                      action="store_true", help="detach and run as daemon [default: no]")
    parser.add_option("-k", "--kill", dest="kill", default=False, action="store_true",
                      help="send SIGTERM to running daemon first [default: no]")
    parser.add_option("-P", "--pidfile", dest="pidfile",
                      default=os.path.join(tempfile.gettempdir(), "sumo-launchd.pid"),
                      help="if running as a daemon, write pid to PIDFILE [default: %default]", metavar="PIDFILE")
    parser.add_option("-t", "--keep-temp", dest="keep_temp", default=False,
                      action="store_true", help="keep all temporary files [default: no]")
    (options, args) = parser.parse_args()
    _LOGLEVELS = (logging.ERROR, logging.WARN, logging.INFO, logging.DEBUG)
    loglevel = _LOGLEVELS[max(
        0, min(1 + options.count_verbose - options.count_quiet, len(_LOGLEVELS) - 1))]

    # catch SIGTERM to exit cleanly when we're kill-ed
    signal.signal(signal.SIGTERM, lambda signum, stack_frame: sys.exit(1))

    # configure logging
    logging.basicConfig(filename=options.logfile, level=loglevel)
    if not options.daemonize:
        logging.getLogger().addHandler(logging.StreamHandler())
    logging.debug("Logging to %s" % options.logfile)

    if args:
        logging.warning(
            "Superfluous command line arguments: \"%s\"" % " ".join(args))

    # this is where we'll spend our time
    wait_for_connections(options.command, options.shlex, options.port, options.bind, options.daemonize, options.kill,
                         options.pidfile, options.keep_temp, options.plugin)


# Start main() when run interactively
if __name__ == '__main__':
    main()