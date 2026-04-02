from openpilot.common.params import Params
from cereal import log
import numpy as np

data = Params().get('LiveTorqueParameters')
if not data:
  print('No LiveTorqueParameters cached')
  exit()

with log.Event.from_bytes(data) as evt:
  lt = evt.liveTorqueParameters
  print('liveValid:  ' + str(lt.liveValid))
  print('calPerc:    ' + str(lt.calPerc) + '%')
  print('totalPts:   ' + str(int(lt.totalBucketPoints)))
  print('F_filtered: ' + str(round(lt.latAccelFactorFiltered, 3)))
  print('F_raw:      ' + str(round(lt.latAccelFactorRaw, 3)))
  print('f_filtered: ' + str(round(lt.frictionCoefficientFiltered, 3)))
  print('f_raw:      ' + str(round(lt.frictionCoefficientRaw, 3)))
  print('resets:     ' + str(int(lt.maxResets)))

  pts = list(lt.points)
  if pts:
    pts_array = np.array(pts).reshape(-1, 2)
    torques = pts_array[:, 0]
    bounds = [(-0.5,-0.3),(-0.3,-0.2),(-0.2,-0.1),(-0.1,0),(0,0.1),(0.1,0.2),(0.2,0.3),(0.3,0.5)]
    mins = [100, 300, 500, 500, 500, 500, 300, 100]
    print('')
    for (lo, hi), need in zip(bounds, mins):
      count = int(np.sum((torques >= lo) & (torques < hi)))
      status = 'OK' if count >= need else 'NEED ' + str(need - count)
      print('  [' + str(lo) + ', ' + str(hi) + ')  count=' + str(count).rjust(5) + '  need=' + str(need).rjust(4) + '  ' + status)
  else:
    print('No cached points data')
