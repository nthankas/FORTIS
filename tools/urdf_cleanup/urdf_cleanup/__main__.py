"""Allow ``python -m urdf_cleanup`` to dispatch to ``main()``."""

from urdf_cleanup.main import main

raise SystemExit(main())
