#!/usr/bin/env python

import sys

from rqt_iris_sami.src.rqt_iris_sami.iris_sami_module import IrisSamiPlugin
from rqt_gui.main import Main


plugin = 'rqt_iris_sami'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))