# Troubleshooting

Format: **Problem → Reason → Fix → Prevention**

## Installation
| Problem | Reason | Fix | Prevention |
|---|---|---|---|
| `'python' is not recognized` (Windows) | Python not on PATH | Reinstall Python and tick **Add python.exe to PATH**, or use `py -3.11 ...` | Use the python.org installer |
| `pip install opencv-python` fails / is very slow | Old pip, unsupported Python (3.13+ on old wheels) or no internet | `python -m pip install --upgrade pip`; use Python 3.10–3.12 | Create the venv with a supported Python |
| `ModuleNotFoundError: No module named 'cv2'` | Venv not activated, or installed in a different Python | Activate the venv, then `python -m pip install -r requirements.txt` | Always run `python -m pip`, not bare `pip` |
| `ModuleNotFoundError: autonav` | Running a script from the wrong folder | Run from the project root: `python main.py` | Scripts add `src/` to the path themselves - don't move them |
| `cannot be loaded because running scripts is disabled` (PowerShell activate) | Execution policy | `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, or use `cmd` and `venv\Scripts\activate.bat` | - |
| NumPy 2 vs OpenCV conflict (`_ARRAY_API not found`) | Old OpenCV build compiled against NumPy 1 | `pip install -U opencv-python numpy` | Install from `requirements.txt` in a fresh venv |

## Simulator window
| Problem | Reason | Fix | Prevention |
|---|---|---|---|
| `--show` opens nothing / `cv2.error ... cvShowImage` | Headless OpenCV build, SSH session, WSL without display, or `opencv-python-headless` installed | `pip uninstall opencv-python-headless opencv-python` then `pip install opencv-python`; or skip `--show` and use `--video` | Never install both OpenCV variants |
| Window freezes | Window not focused / heavy compute | Click the window; run with `--quiet` | - |
| Video won't play in the browser | `mp4v` codec | Install ffmpeg (the code auto-converts to H.264) or use the GIF | `ffmpeg -version` to check |

## Perception ("model not loading")
| Problem | Reason | Fix | Prevention |
|---|---|---|---|
| Obstacles not detected | HSV ranges don't match your changed colours | Edit `STATIC_HSV`/`DYNAMIC_HSV` in `perception.py`; run `tests/test_perception.py` | Keep colours in `world.py` and ranges in `perception.py` in sync |
| Plenty of false detections | Too much noise / too-small `MIN_BLOB_PX` | Raise `MIN_BLOB_PX` or lower `cam_noise_std` | Print `len(det.detections)` per frame while tuning |
| YOLO weights file not found (`yolov3.weights`) *(when you upgrade to YOLO)* | Weights are not in the repo (large files) | Download once to `models/` and load via a path relative to the project | Document the download command in the README |

## Planning
| Problem | Reason | Fix | Prevention |
|---|---|---|---|
| "no path" / robot stands still in `WAITING FOR PATH` | Gap narrower than robot + margin, or goal inside an inflated zone | Lower `inflation_margin` or `robot_radius`; widen the gap | Run `scenarios.is_solvable(world)` when authoring maps |
| Path cuts through a wall corner | Diagonal moves allowed between two blocked cells | Already prevented in `_valid_move` (test: `test_no_diagonal_corner_cutting`) | Keep the unit test |
| Planner is slow | Grid too fine | Increase `cell_m` (e.g. 0.5) | Watch `avg_plan_ms` in the metrics |
| Robot zig-zags | Raw grid path | Make sure `smooth_path` is used | - |

## Collision / safety logic
| Problem | Reason | Fix | Prevention |
|---|---|---|---|
| Robot freezes next to an obstacle | Safety rule triggered by a *sideways* obstacle (we hit this during development!) | Only stop for obstacles in the direction of travel; always allow rotation | Test with a pallet next to the start |
| Robot grazes moving obstacles | Planner only sees where things *are* | Keep the tracking/prediction layer on (`avoidance.py`) | Check `min_clearance_m` in `metrics.json` |
| Collisions with `collisions: 0` but clearance ≈ 0 | Pixel quantisation (5 cm) | Treat `min_clearance_m < 0.05` as a near-miss | Report clearance, not only collisions |

## Git / GitHub
| Problem | Reason | Fix | Prevention |
|---|---|---|---|
| `fatal: not a git repository` | Not inside the project folder / no `git init` | `cd` into the folder, `git init` | - |
| `fatal: remote origin already exists` | Remote was added twice | `git remote set-url origin <URL>` | - |
| `error: failed to push some refs` | GitHub repo has a README/licence you don't have locally | `git pull origin main --allow-unrelated-histories`, resolve, push | Create the GitHub repo **empty** |
| `Permission denied (publickey)` / `Authentication failed` | Password logins were removed | Use HTTPS + a **Personal Access Token**, or the GitHub CLI `gh auth login` | Use `gh auth login` once |
| `src refspec main does not match any` | No commit yet, or branch is named `master` | `git add . && git commit -m "..."` then `git branch -M main` | Commit before pushing |
| File over 100 MB rejected | GitHub's limit | Keep videos short; use a GIF; host long videos on YouTube and link them | `.gitignore` large outputs |
| Images don't show in README | Wrong relative path or case (`Images/` vs `images/`) | Use `images/name.png` exactly (Linux is case-sensitive) | Preview README on GitHub after each push |
