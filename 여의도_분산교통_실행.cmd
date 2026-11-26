@echo off
cd /d "%~dp0"
python -m inference.fixed_population_panel %*
