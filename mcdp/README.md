
To use the docker version do:

    alias docker-mcdp-solve='docker run --rm  -v ${PWD}:${PWD} -w ${PWD} registry-stage.duckietown.org/andreacensi/mcdp:z6-py38 mcdp-solve'

Then:

    docker-mcdp-solve -d ral2020.mcdplib lane_cameras '<1 pixel/steradian, 1 Hz, `timeday: day>'
