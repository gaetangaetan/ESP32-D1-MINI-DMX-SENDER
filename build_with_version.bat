@echo off
REM Script de build complet avec mise à jour automatique de la version
REM Usage: build_with_version.bat [timestamp_optionnel]

echo ========================================
echo    BUILD KSOLOTI KONTROL ESP32
echo ========================================
echo.

REM Vérifier si Python est disponible
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ❌ Python n'est pas installé ou pas dans le PATH
    echo    Veuillez installer Python pour utiliser la mise à jour automatique de version
    echo.
    echo 🔄 Build sans mise à jour de version...
    goto :build_only
)

REM Mettre à jour la version si un argument est fourni
if "%1" neq "" (
    echo 🔄 Mise à jour de la version vers: %1
    python update_version.py %1
    if %errorlevel% neq 0 (
        echo ❌ Erreur lors de la mise à jour de la version
        pause
        exit /b 1
    )
    echo.
) else (
    echo 🔄 Génération automatique de la version...
    python update_version.py
    if %errorlevel% neq 0 (
        echo ❌ Erreur lors de la génération de la version
        pause
        exit /b 1
    )
    echo.
)

:build_only
echo 🔨 Compilation du firmware...
pio run
if %errorlevel% neq 0 (
    echo ❌ Erreur de compilation
    pause
    exit /b 1
)

echo.
echo 📁 Construction du système de fichiers LittleFS...
pio run --target buildfs
if %errorlevel% neq 0 (
    echo ❌ Erreur de construction LittleFS
    pause
    exit /b 1
)

echo.
echo ✅ Build terminé avec succès !
echo.
echo 📦 Fichiers générés :
dir .pio\build\wemos_d1_mini32\*.bin

echo.
echo 🚀 Prêt pour l'upload OTA !
echo    - firmware.bin: %~dp0.pio\build\wemos_d1_mini32\firmware.bin
echo    - littlefs.bin: %~dp0.pio\build\wemos_d1_mini32\littlefs.bin
echo.
pause
