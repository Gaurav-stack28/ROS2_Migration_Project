git add .

if (-not (git diff --cached --quiet)) {
    git commit -m "Update project"

    if ($LASTEXITCODE -eq 0) {
        git push
    }
}
else {
    Write-Host "No changes to commit."
}

git status
