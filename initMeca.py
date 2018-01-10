from Lib.models import *
from initPropulsion import *


# construct mecanical model
# an engine with propeller at the end of an arm aligned with y and rotating around x
#      z
#     ^
#     |   --|-- propeller
#     o-----M engine1
#     x     y

armLength = 0.5

# Engine + propeller is modelled as a point mass.
# We add an arbitrary mass that the engine is capable of lifting
arbitraryMass = 0.18

d = {'mass': engine.getMass() + propeller.getMass() + arbitraryMass,
'xG': 0.,
'yG': 0.5 * armLength,
'zG': 0.,
}

pmM1 = Point_mass(d)

quadri = Meca_model()
quadri.add(pmM1)

# rotation axis
axe = (1, 0, 0)