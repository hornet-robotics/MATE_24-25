import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jbeta/Software/mate_software/mate_workspace/install/simple_package'
