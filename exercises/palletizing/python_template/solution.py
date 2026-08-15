"""Pick incoming boxes and place them using HAL-provided task data."""

import HAL


def main():
    pallet = HAL.GetPalletInfo()

    box_name = HAL.WaitForBox()
    box = HAL.GetBoxInfo(box_name)
    pickup = HAL.GetPickupPose(box_name)

    print(f"Ready box: {box}")
    print(f"Pickup pose: {pickup}")
    print(f"Pallet: {pallet}")

    # TODO: Pick the box and acknowledge it once clear of the conveyor.
    # TODO: Choose an in-bounds target from the box and pallet data.
    # TODO: Place and release the box, then retreat safely.


if __name__ == "__main__":
    main()
