import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jbeta/Software/mate_software/mate_workspace/src/install/simple_package'
