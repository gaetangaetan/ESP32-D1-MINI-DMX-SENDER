#!/usr/bin/env python3
"""
Script pour mettre à jour la version dans tous les fichiers du projet
Usage: python update_version.py [nouveau_timestamp]
Si aucun timestamp n'est fourni, génère automatiquement le timestamp actuel
"""

import sys
import re
import time
import os

def get_current_timestamp():
    """Génère le timestamp Unix actuel"""
    return int(time.time())

def update_version_in_file(file_path, old_version, new_version):
    """Met à jour la version dans un fichier"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Remplacer toutes les occurrences de l'ancienne version
        updated_content = content.replace(str(old_version), str(new_version))
        
        if updated_content != content:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(updated_content)
            print(f"✅ {file_path} mis à jour")
            return True
        else:
            print(f"⚠️  {file_path} - aucune occurrence trouvée")
            return False
            
    except Exception as e:
        print(f"❌ Erreur lors de la mise à jour de {file_path}: {e}")
        return False

def main():
    # Déterminer la nouvelle version
    if len(sys.argv) > 1:
        try:
            new_version = int(sys.argv[1])
        except ValueError:
            print("❌ Erreur: Le timestamp doit être un nombre entier")
            sys.exit(1)
    else:
        new_version = get_current_timestamp()
        print(f"🕐 Timestamp généré automatiquement: {new_version}")
    
    print(f"🔄 Mise à jour vers la version: {new_version}")
    
    # Fichiers à mettre à jour
    files_to_update = [
        'src/main.cpp',
        'data/index.html', 
        'data/script.js'
    ]
    
    # Version actuelle (à extraire du premier fichier)
    current_version = None
    try:
        with open('src/main.cpp', 'r', encoding='utf-8') as f:
            content = f.read()
            match = re.search(r'#define VERSION (\d+)', content)
            if match:
                current_version = int(match.group(1))
                print(f"📋 Version actuelle détectée: {current_version}")
            else:
                print("⚠️  Version actuelle non trouvée dans main.cpp")
    except Exception as e:
        print(f"❌ Erreur lors de la lecture de main.cpp: {e}")
        sys.exit(1)
    
    if current_version is None:
        print("❌ Impossible de déterminer la version actuelle")
        sys.exit(1)
    
    # Mettre à jour tous les fichiers
    success_count = 0
    for file_path in files_to_update:
        if os.path.exists(file_path):
            if update_version_in_file(file_path, current_version, new_version):
                success_count += 1
        else:
            print(f"⚠️  Fichier non trouvé: {file_path}")
    
    print(f"\n🎉 Mise à jour terminée: {success_count}/{len(files_to_update)} fichiers mis à jour")
    print(f"📅 Nouvelle version: {new_version}")
    print(f"📅 Date correspondante: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(new_version))}")

if __name__ == "__main__":
    main()
