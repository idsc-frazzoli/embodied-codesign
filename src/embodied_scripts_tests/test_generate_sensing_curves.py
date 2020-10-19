from decimal import Decimal

from nose.tools import assert_equal

from embodied_scripts.generate_sensing_curves import get_vertical_resolution, get_recall_precision, get_accuracy


def get_vertical_resolution_test() -> None:
    res_pix_test = [1000, 1000]
    chip_size_test = [0.01, 0.01]
    cam_test = {"chip_size_m": chip_size_test, "focal_length_m": 0.01, "resolution_pxl": res_pix_test}
    list_of_ds = [Decimal('0.0'), Decimal('0.2'), Decimal('2.0'), Decimal('100.0')]
    h_ped = Decimal('1.65')
    v_resolution = get_vertical_resolution(list_of_ds, cam_test, h_ped)
    v_resolution_expect = [Decimal('1000.0'), Decimal('1000.0'), Decimal('825.0'), Decimal('16.5')]

    assert_equal(v_resolution, v_resolution_expect)

def get_accuracy_test() -> None:
    res_pix_test = [1000, 1000]
    chip_size_test = [0.01, 0.01]
    cam_test = {"chip_size_m": chip_size_test, "focal_length_m": 0.01, "resolution_pxl": res_pix_test}
    list_of_ds = [Decimal('0.0'), Decimal('0.2'), Decimal('2.0'), Decimal('100.0')]
    accuracy = get_accuracy(cam_test, list_of_ds)
    accuracy_expect = [0.0, 0.00016, 0.016, 40.0]

    assert_equal(accuracy, accuracy_expect)

