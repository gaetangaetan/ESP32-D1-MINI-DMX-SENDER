# Script de build complet avec mise à jour automatique de la version
# Usage: .\build_with_version.ps1 [timestamp_optionnel]

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "    BUILD KSOLOTI KONTROL ESP32" -ForegroundColor Cyan  
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Vérifier si Python est disponible
try {
    $pythonVersion = python --version 2>&1
    Write-Host "✅ Python détecté: $pythonVersion" -ForegroundColor Green
} catch {
    Write-Host "❌ Python n'est pas installé ou pas dans le PATH" -ForegroundColor Red
    Write-Host "   Veuillez installer Python pour utiliser la mise à jour automatique de version" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "🔄 Build sans mise à jour de version..." -ForegroundColor Yellow
    $skipVersionUpdate = $true
}

if (-not $skipVersionUpdate) {
    # Mettre à jour la version si un argument est fourni
    if ($args.Count -gt 0) {
        $timestamp = $args[0]
        Write-Host "🔄 Mise à jour de la version vers: $timestamp" -ForegroundColor Yellow
        python update_version.py $timestamp
        if ($LASTEXITCODE -ne 0) {
            Write-Host "❌ Erreur lors de la mise à jour de la version" -ForegroundColor Red
            Read-Host "Appuyez sur Entrée pour continuer"
            exit 1
        }
    } else {
        Write-Host "🔄 Génération automatique de la version..." -ForegroundColor Yellow
        python update_version.py
        if ($LASTEXITCODE -ne 0) {
            Write-Host "❌ Erreur lors de la génération de la version" -ForegroundColor Red
            Read-Host "Appuyez sur Entrée pour continuer"
            exit 1
        }
    }
    Write-Host ""
}

Write-Host "🔨 Compilation du firmware..." -ForegroundColor Yellow
pio run
if ($LASTEXITCODE -ne 0) {
    Write-Host "❌ Erreur de compilation" -ForegroundColor Red
    Read-Host "Appuyez sur Entrée pour continuer"
    exit 1
}

Write-Host ""
Write-Host "📁 Construction du système de fichiers LittleFS..." -ForegroundColor Yellow
pio run --target buildfs
if ($LASTEXITCODE -ne 0) {
    Write-Host "❌ Erreur de construction LittleFS" -ForegroundColor Red
    Read-Host "Appuyez sur Entrée pour continuer"
    exit 1
}

Write-Host ""
Write-Host "✅ Build terminé avec succès !" -ForegroundColor Green
Write-Host ""
Write-Host "📦 Fichiers générés :" -ForegroundColor Cyan
Get-ChildItem .pio\build\wemos_d1_mini32\*.bin | Format-Table Name, Length, LastWriteTime -AutoSize

Write-Host ""
Write-Host "🚀 Prêt pour l'upload OTA !" -ForegroundColor Green
Write-Host "   - firmware.bin: $PWD\.pio\build\wemos_d1_mini32\firmware.bin" -ForegroundColor Gray
Write-Host "   - littlefs.bin: $PWD\.pio\build\wemos_d1_mini32\littlefs.bin" -ForegroundColor Gray
Write-Host ""
Read-Host "Appuyez sur Entrée pour continuer"
