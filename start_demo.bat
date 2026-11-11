@echo off
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -Command "$s=Get-Content -LiteralPath '.\demo_settings.json' -Raw | ConvertFrom-Json; & $s.idf_python '.\tools\app.py'"
if errorlevel 1 pause
