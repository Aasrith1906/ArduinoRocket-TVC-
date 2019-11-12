#!"E:\Code\git reps\ArduinoRocket(TVC)\ArduinoRocket-TVC-\FlightAnalysisPython\venv\Scripts\python.exe"
# EASY-INSTALL-ENTRY-SCRIPT: 'anaconda-client==1.2.2','console_scripts','anaconda'
__requires__ = 'anaconda-client==1.2.2'
import re
import sys
from pkg_resources import load_entry_point

if __name__ == '__main__':
    sys.argv[0] = re.sub(r'(-script\.pyw?|\.exe)?$', '', sys.argv[0])
    sys.exit(
        load_entry_point('anaconda-client==1.2.2', 'console_scripts', 'anaconda')()
    )
