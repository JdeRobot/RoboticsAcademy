# Pick and Place Harmonic - Object Coordinates Reference

## Object Positions on Conveyor (base_link frame)

Based on warehouse.sdf spawned objects:

### Red Box
- **Pick**: X=0.6, Y=-0.3, Z=0.265
- **Width**: 0.055m (55mm)
- **Drop**: Red bin at X=-0.4, Y=0.15, Z=0.0

### Yellow Box  
- **Pick**: X=0.6, Y=0.3, Z=0.275
- **Width**: 0.045m (45mm)
- **Drop**: Yellow bin at X=-0.4, Y=-0.45, Z=0.0

### Blue Ball
- **Pick**: X=0.7, Y=0.1, Z=0.265
- **Width**: 0.085m (85mm - full open for sphere)
- **Drop**: Blue bin at X=-0.4, Y=0.45, Z=0.0

### Green Cylinder
- **Pick**: X=0.5, Y=-0.1, Z=0.295
- **Width**: 0.065m (65mm)
- **Drop**: Green bin at X=-0.4, Y=-0.15, Z=0.0

## Pick Strategy

1. **Pre-pick**: Add 0.10m to Z (10cm above object)
2. **Pick**: Use exact Z coordinate
3. **Grasp**: Use object width with HAL.grasp()
4. **Lift**: Return to pre-pick height
5. **Home**: Move to [0.0, -0.4, 0.4]
6. **Pre-drop**: Bin position + 0.25m in Z
7. **Drop**: Remove bin collision, lower to bin Z
8. **Release**: Open gripper

## Example Code

```python
# Red Box Pick
pre_pick = [0.6, -0.3, 0.365]  # 10cm above
pick = [0.6, -0.3, 0.265]      # Exact height
HAL.move_to_pose(*pre_pick)
HAL.move_to_pose(*pick)
HAL.grasp(width=0.055)
HAL.move_to_pose(*pre_pick)

# Red Box Drop
HAL.remove_collision_object('red_bin')
drop = [-0.4, 0.15, 0.0]
HAL.move_to_pose(*drop)
HAL.open_gripper()
```

## Coordinate Frame Reference

- **base_link**: Robot base frame
- **World Z=0.9**: Robot base is 0.9m above ground
- **base_link Z**: Relative to robot base
  - Conveyor surface: Z=0.05
  - Table surface: Z=-0.17
  - Bin tops: Z=-0.145
  - Objects: Z=0.14 to 0.17 (varies by object)

## Safe Positions

- **Home**: [0.0, -0.4, 0.4] - High, back position
- **Pre-pick range**: Z=0.3 to 0.4 above conveyor
- **Pre-drop range**: Z=0.2 to 0.3 above bins
