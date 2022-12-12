#!/bin/bash

xacro ../../../moma_description/urdf/panda.urdf.xacro | sed -e 's/package:\///' > robodash/panda.urdf
xacro ../../../moma_description/urdf/superpanda.urdf.xacro | sed -e 's/package:\///' > robodash/smb.urdf
