import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dex/dexe_mobile_nav/mobile_simulation/swerve-sim-container/install/swerve_controller'
