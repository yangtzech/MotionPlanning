def test_imports():
    import pypnc
    from pypnc import utils
    from pypnc.controllers import pure_pursuit

    # basic smoke tests
    assert hasattr(pypnc, 'State')
    assert hasattr(utils, 'pi_2_pi')
    # pure_pursuit module should expose the controller function
    assert hasattr(pure_pursuit, 'pure_pursuit') or callable(pure_pursuit)
