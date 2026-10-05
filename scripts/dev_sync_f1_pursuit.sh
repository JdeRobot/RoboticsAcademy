#!/bin/bash
# Push the f1_pursuit exercise into a running RADI container and build what
# needs building, so the exercise can be tested without rebuilding the image.
#
# The user mode compose files mount nothing, so everything this copies lives
# only in the running container and is lost when the container is recreated.
# Re-run it after every `run_academy.sh`.
#
#   ./scripts/dev_sync_f1_pursuit.sh            # sync, build, load db rows
#   ./scripts/dev_sync_f1_pursuit.sh -s         # skip the colcon and C++ builds
#   ./scripts/dev_sync_f1_pursuit.sh -f         # skip the react bundle rebuild
#   ./scripts/dev_sync_f1_pursuit.sh -x         # overwrite the editor code with the sample
#   ./scripts/dev_sync_f1_pursuit.sh -c mybox   # a differently named container
#
# RI_DIR overrides where RoboticsInfrastructure is looked for.

set -euo pipefail

CONTAINER="developer-container"
DB_CONTAINER="world_db"
SKIP_BUILD="false"
SKIP_FRONTEND="false"
SEED_FORCE="false"
EXERCISE="f1_pursuit"

while getopts ":c:sfxh" opt; do
  case $opt in
    c) CONTAINER="$OPTARG" ;;
    s) SKIP_BUILD="true" ;;
    f) SKIP_FRONTEND="true" ;;
    x) SEED_FORCE="true" ;;
    h) sed -n '2,14p' "$0"; exit 0 ;;
    \?) echo "Error: invalid option -$OPTARG" >&2; exit 1 ;;
  esac
done

RA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Prefer a standalone RoboticsInfrastructure checkout next to RoboticsAcademy,
# fall back to the submodule inside it.
if [ -n "${RI_DIR:-}" ]; then
  :
elif [ -d "$RA_DIR/../RoboticsInfrastructure/Launchers" ]; then
  RI_DIR="$(cd "$RA_DIR/../RoboticsInfrastructure" && pwd)"
elif [ -d "$RA_DIR/RoboticsInfrastructure/Launchers" ]; then
  RI_DIR="$RA_DIR/RoboticsInfrastructure"
else
  echo "Error: no RoboticsInfrastructure checkout found. Set RI_DIR." >&2
  exit 1
fi

say() { printf '\n==> %s\n' "$1"; }

# ---------------------------------------------------------------- preflight
if ! docker info >/dev/null 2>&1; then
  echo "Error: the Docker daemon is not reachable." >&2
  exit 1
fi
for c in "$CONTAINER" "$DB_CONTAINER"; do
  if [ -z "$(docker ps -q -f "name=^${c}$")" ]; then
    echo "Error: container '$c' is not running. Start the academy first:" >&2
    echo "  cd $RA_DIR && ./scripts/run_academy.sh -n" >&2
    exit 1
  fi
done

for f in \
  "$RI_DIR/Launchers/f1_pursuit_simple.launch.py" \
  "$RI_DIR/Scenes/f1_pursuit_simple.world" \
  "$RI_DIR/CustomRobots/f1/models/simple_circuit_wide/model.sdf" \
  "$RI_DIR/resources/exercises/$EXERCISE/rival.py" \
  "$RA_DIR/exercises/$EXERCISE/python_template/HAL.py" \
  "$RA_DIR/exercises/$EXERCISE/cpp_lib/CMakeLists.txt"
do
  [ -f "$f" ] || { echo "Error: missing $f" >&2; exit 1; }
done

echo "container:  $CONTAINER"
echo "academy:    $RA_DIR"
echo "infra:      $RI_DIR"

# ------------------------------------------------------- world and launchers
say "Copying launchers, visualization config and world"
for f in "$RI_DIR"/Launchers/f1_pursuit_simple*.launch.py; do
  docker cp "$f" "$CONTAINER:/opt/jderobot/Launchers/$(basename "$f")"
done
docker cp "$RI_DIR/Launchers/visualization/f1_pursuit.config" \
          "$CONTAINER:/opt/jderobot/Launchers/visualization/f1_pursuit.config"
docker cp "$RI_DIR/Scenes/f1_pursuit_simple.world" \
          "$CONTAINER:/opt/jderobot/Scenes/f1_pursuit_simple.world"

# ------------------------------------------------------------- robot sources
say "Copying the F1 launch file, xacros and the widened circuit model"
docker cp "$RI_DIR/CustomRobots/f1/launch/f1.launch.py" \
          "$CONTAINER:/home/ws/src/CustomRobots/f1/launch/f1.launch.py"
for f in f1.urdf.xacro f1_common.urdf.xacro f1_gz.urdf.xacro; do
  docker cp "$RI_DIR/CustomRobots/f1/models/f1/$f" \
            "$CONTAINER:/home/ws/src/CustomRobots/f1/models/f1/$f"
done
docker exec "$CONTAINER" rm -rf /home/ws/src/CustomRobots/f1/models/simple_circuit_wide
docker cp "$RI_DIR/CustomRobots/f1/models/simple_circuit_wide" \
          "$CONTAINER:/home/ws/src/CustomRobots/f1/models/simple_circuit_wide"

if [ "$SKIP_BUILD" = "true" ]; then
  say "Skipping the colcon build (-s)"
else
  say "Building custom_robots so the new model and xacros get installed"
  docker exec "$CONTAINER" bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    cd /home/ws
    colcon build --packages-select custom_robots --cmake-args -DCMAKE_BUILD_TYPE=Release
  '
fi

# --------------------------------------------------------- rival entrypoint
say "Copying the rival entrypoint"
docker exec "$CONTAINER" mkdir -p "/resources/exercises/$EXERCISE"
docker cp "$RI_DIR/resources/exercises/$EXERCISE/rival.py" \
          "$CONTAINER:/resources/exercises/$EXERCISE/rival.py"

# ------------------------------------------------------- exercise templates
say "Copying the exercise templates"
docker exec "$CONTAINER" rm -rf "/RoboticsAcademy/exercises/$EXERCISE"
docker cp "$RA_DIR/exercises/$EXERCISE" "$CONTAINER:/RoboticsAcademy/exercises/$EXERCISE"

# --------------------------------------------------------------- front end
# Project.tsx does import(`exercises/${id}/frontend/WebGUI.tsx`), which webpack
# turns into a context module resolved at BUILD time. The image ships one chunk
# per exercise, so a brand new exercise has no chunk and the GUI panel fails
# with "WebGUI.js failed to load". The bundle has to be rebuilt and pushed in.
# The container has no node and no src, so this builds on the host.
if [ "$SKIP_FRONTEND" = "true" ]; then
  say "Skipping the react bundle rebuild (-f)"
else
  say "Rebuilding the react bundle so the exercise GUI has a chunk"

  if ! command -v node >/dev/null 2>&1; then
    echo "  node not found on the host. Install node 22+ and re-run, or pass -f" >&2
    exit 1
  fi
  node_major="$(node -p 'process.versions.node.split(".")[0]')"
  if [ "$node_major" -lt 22 ]; then
    echo "  node $(node --version) is too old: camera-controls needs >= 22." >&2
    echo "  Point PATH at a newer node (nvm use 24) and re-run, or pass -f." >&2
    exit 1
  fi

  if [ ! -d "$RA_DIR/react_frontend/node_modules" ]; then
    echo "  Installing front end dependencies (first run only)"
    ( cd "$RA_DIR/react_frontend" && yarn install --frozen-lockfile )
  fi

  # Generated artefact, same steps as scripts/RADI/Dockerfile.humble. It is the
  # zip of python libs that ships to the student alongside their code, and the
  # bundle will not compile without it.
  if [ ! -f "$RA_DIR/react_frontend/src/common.zip" ]; then
    echo "  Building common.zip"
    # cd INTO each package dir, as Dockerfile.humble does: zipping from
    # common/ instead nests them as hal_interfaces/hal_interfaces/... and the
    # student's `from hal_interfaces.general...` import fails at runtime.
    ( cd "$RA_DIR/common"
      rm -f ../common.zip common.zip
      ( cd console_interfaces && zip -rq  ../../common.zip console_interfaces/ )
      ( cd gui_interfaces     && zip -rqu ../../common.zip gui_interfaces/ )
      ( cd hal_interfaces     && zip -rqu ../../common.zip hal_interfaces/ )
      mv ../common.zip ../react_frontend/src/common.zip )
  fi

  ( cd "$RA_DIR/react_frontend" && yarn build )

  chunk=$(ls "$RA_DIR/react_frontend/static/react_frontend/js" | grep "exercises_${EXERCISE}_frontend_WebGUI_tsx" | head -1)
  [ -n "$chunk" ] || { echo "  FAIL webpack produced no chunk for $EXERCISE" >&2; exit 1; }
  echo "  built $chunk"

  docker exec "$CONTAINER" rm -rf /RoboticsAcademy/react_frontend/static/react_frontend
  docker cp "$RA_DIR/react_frontend/static/react_frontend" \
            "$CONTAINER:/RoboticsAcademy/react_frontend/static/react_frontend"
  docker cp "$RA_DIR/react_frontend/webpack-stats.json" \
            "$CONTAINER:/RoboticsAcademy/react_frontend/webpack-stats.json"
fi

# -------------------------------------------------------------- sample code
# enter_exercise creates academy.py and academy.cpp empty the first time the
# exercise is opened. Seed them from solution/ so there is something to press
# play on, but never clobber code that is already there.
say "Seeding the sample solution into the editor"
for pair in "academy.py:solution/academy.py" "academy.cpp:solution/academy.cpp"; do
  name="${pair%%:*}"; src="$RA_DIR/exercises/$EXERCISE/${pair##*:}"
  dst="/RoboticsAcademy/filesystem/$EXERCISE/$name"
  [ -f "$src" ] || continue
  existing=$(docker exec "$CONTAINER" bash -lc "wc -c < '$dst' 2>/dev/null || echo 0")
  if [ "$SEED_FORCE" = "true" ] || [ "${existing:-0}" -lt 5 ]; then
    docker exec "$CONTAINER" mkdir -p "/RoboticsAcademy/filesystem/$EXERCISE"
    docker cp "$src" "$CONTAINER:$dst"
    echo "  seeded $name"
  else
    echo "  kept your $name ($existing bytes); -x overwrites it"
  fi
done

# ------------------------------------------------------------ C++ libraries
# compile_exercise.sh is not in the image, so its steps run here. The .so files
# ship to the student inside the code zip, so they are copied back to the repo.
if [ "$SKIP_BUILD" = "true" ]; then
  say "Skipping the C++ library build (-s)"
else
  say "Building the C++ exercise libraries"
  docker exec "$CONTAINER" bash -lc "
    set -e
    source /opt/ros/humble/setup.bash
    source /home/ws/install/setup.bash
    cd /RoboticsAcademy/exercises/$EXERCISE/cpp_lib
    rm -rf build && mkdir build && cd build
    cmake .. >/dev/null
    make -j\$(nproc)
    strip --strip-unneeded *.so
    chmod 777 *.so
    mkdir -p ../../cpp_template/libs
    mv *.so ../../cpp_template/libs/
    cd .. && rm -rf build
    cp -r include/. ../cpp_template/libs/include/
  "
  say "Copying the built libraries back into the repo"
  for lib in libHAL.so libWebGUI.so libFrequency.so; do
    docker cp "$CONTAINER:/RoboticsAcademy/exercises/$EXERCISE/cpp_template/libs/$lib" \
              "$RA_DIR/exercises/$EXERCISE/cpp_template/libs/$lib"
  done
fi

# --------------------------------------------------------------- database
# Re-running the pg_dump files would collide on the primary keys, so only the
# f1_pursuit rows are loaded, and they are deleted first to stay idempotent.
say "Loading the f1_pursuit database rows"
docker exec -i "$DB_CONTAINER" psql -q -v ON_ERROR_STOP=1 -U user-dev -d academy_db <<'SQL'
BEGIN;

DELETE FROM exercises_tools  WHERE exercise_id = 36;
DELETE FROM exercises_worlds WHERE exercise_id = 36;
DELETE FROM exercises        WHERE id = 36;
DELETE FROM worlds_robots    WHERE world_id IN (92);
DELETE FROM worlds           WHERE id IN (92);
DELETE FROM robots           WHERE id IN (43, 44);
DELETE FROM scenes           WHERE id IN (84);

INSERT INTO scenes (id, name, launch_file_path, tools_config, ros_version, type, model_path) VALUES
 (84,'F1 Pursuit Simple','/opt/jderobot/Launchers/f1_pursuit_simple.launch.py','{"gzsim":"/opt/jderobot/Launchers/visualization/f1_pursuit.config"}','ROS2','gz','simple_circuit.urdf');

INSERT INTO robots (id, name, launch_file_path, entity, extra_config, model_path) VALUES
 (43,'F1 Pursuit Chaser','/home/ws/src/CustomRobots/f1/launch/f1.launch.py','f1','mode:=holo sensor:=camera namespace:=f1 color:=F1Blue','f1/models/f1/f1.urdf.xacro'),
 (44,'F1 Pursuit Rival','/home/ws/src/CustomRobots/f1/launch/f1.launch.py','f1_rival','mode:=holo sensor:=camera namespace:=f1_rival color:=F1Magenta','f1/models/f1/f1.urdf.xacro');

INSERT INTO worlds (id, name, scene_id) VALUES
 (92,'F1 Pursuit Simple',84);

-- Abreast on the start straight, each car on its own painted line
INSERT INTO worlds_robots (id, world_id, robot_id, poses) VALUES
 (68,92,43,'{{85.606,-17.414,0.006,0.0,0.0,-1.571}}'),
 (69,92,44,'{{86.726,-17.414,0.006,0.0,0.0,-1.571}}');

INSERT INTO exercises (id, exercise_id, name, description, tags, entrypoints, status, url) VALUES
 (36,'f1_pursuit','Formula 1 Pursuit',
  'Two Formula 1 cars on a racing circuit: program your car to chase down the pre programmed rival driving its own line',
  '["ROS2","AUTONOMOUS DRIVING", "MULTILANGUAGE"]',
  '["/resources/exercises/f1_pursuit/rival.py"]',
  'ACTIVE','https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/f1_pursuit');

INSERT INTO exercises_worlds (id, exercise_id, world_id, is_default) VALUES
 (93,36,92,True);

INSERT INTO exercises_tools (id, exercise_id, tool_id) VALUES
 (110,36,'console'),(111,36,'simulator'),(112,36,'web_gui');

COMMIT;
SQL

# ----------------------------------------------------------------- verify
say "Verifying"

docker exec "$CONTAINER" bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ws/install/setup.bash
  share=$(ros2 pkg prefix custom_robots)/share/custom_robots
  test -f "$share/models/simple_circuit_wide/model.sdf" \
    && echo "  ok   widened circuit model installed" \
    || { echo "  FAIL widened circuit model not installed"; exit 1; }
  # each car follows its own painted line, so the generated lane has to be there
  test -s "$share/models/simple_circuit_wide/meshes/rival_lane.obj" \
    && echo "  ok   rival lane mesh installed" \
    || { echo "  FAIL rival lane mesh not installed"; exit 1; }
  grep -q "rival_lane" "$share/models/simple_circuit_wide/model.sdf" \
    && echo "  ok   the circuit model paints the rival lane" \
    || { echo "  FAIL model.sdf does not reference the rival lane"; exit 1; }
  # the colour argument has to survive xacro, or both cars look identical
  xacro "$share/models/f1/f1.urdf.xacro" namespace:=f1_rival color:=F1Magenta \
    > /tmp/f1_rival.urdf 2>/tmp/f1_rival.err \
    && echo "  ok   xacro processes with color:=F1Magenta" \
    || { echo "  FAIL xacro rejected the colour argument:"; cat /tmp/f1_rival.err; exit 1; }
  grep -q "F1Magenta" /tmp/f1_rival.urdf \
    && echo "  ok   the material reaches the generated urdf" \
    || { echo "  FAIL no material in the generated urdf"; exit 1; }
  grep -q "/f1_rival/cmd_vel" /tmp/f1_rival.urdf \
    && echo "  ok   the rival topics are namespaced" \
    || { echo "  FAIL rival topics are not namespaced"; exit 1; }
  # and with no colour the old worlds must be untouched
  xacro "$share/models/f1/f1.urdf.xacro" > /tmp/f1_plain.urdf 2>/dev/null \
    && echo "  ok   xacro still processes with no colour (follow_line unaffected)" \
    || { echo "  FAIL plain xacro broke"; exit 1; }
'

docker exec "$CONTAINER" bash -lc '
  for f in /opt/jderobot/Launchers/f1_pursuit_simple.launch.py \
           /opt/jderobot/Scenes/f1_pursuit_simple.world \
           /resources/exercises/f1_pursuit/rival.py; do
    test -f "$f" && echo "  ok   $f" || { echo "  FAIL missing $f"; exit 1; }
  done
  python3 -c "import ast,sys; ast.parse(open(\"/resources/exercises/f1_pursuit/rival.py\").read())" \
    && echo "  ok   rival.py parses" || { echo "  FAIL rival.py does not parse"; exit 1; }
  python3 -c "import ast,sys; ast.parse(open(\"/RoboticsAcademy/exercises/f1_pursuit/solution/academy.py\").read())" \
    && echo "  ok   the python sample solution parses" \
    || { echo "  FAIL the python sample solution does not parse"; exit 1; }
'

if [ "$SKIP_BUILD" != "true" ]; then
  docker exec "$CONTAINER" bash -lc '
    for l in libHAL.so libWebGUI.so libFrequency.so; do
      f=/RoboticsAcademy/exercises/f1_pursuit/cpp_template/libs/$l
      test -s "$f" && echo "  ok   built $l" || { echo "  FAIL $l missing"; exit 1; }
    done
  '
fi

if [ "$SKIP_FRONTEND" != "true" ]; then
  docker exec "$CONTAINER" bash -lc '
    js=/RoboticsAcademy/react_frontend/static/react_frontend/js
    ls $js | grep -q "exercises_f1_pursuit_frontend_WebGUI_tsx" \
      && echo "  ok   the exercise gui chunk is in the container" \
      || { echo "  FAIL no gui chunk in the container"; exit 1; }
    grep -qoE "\./f1_pursuit/frontend/WebGUI\.tsx" $js/main.*.js \
      && echo "  ok   main.js resolves the exercise gui" \
      || { echo "  FAIL main.js has no entry for the exercise"; exit 1; }
  '
fi

rows=$(docker exec "$DB_CONTAINER" psql -tAc "
  SELECT (SELECT count(*) FROM exercises WHERE exercise_id='f1_pursuit')
       ||'/'|| (SELECT count(*) FROM exercises_worlds WHERE exercise_id=36)
       ||'/'|| (SELECT count(*) FROM worlds_robots WHERE world_id IN (92));
" -U user-dev -d academy_db)
echo "  ok   db rows exercise/worlds/robot-placements = $rows (expect 1/1/2)"

say "Done. Hard reload the browser (ctrl-shift-R) and pick 'Formula 1 Pursuit'."
echo "    http://127.0.0.1:7164/academy/"
