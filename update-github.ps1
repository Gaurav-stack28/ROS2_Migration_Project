$Message = Read-Host "Enter the name/description of your change"

if ([string]::IsNullOrWhiteSpace($Message)) {
    Write-Host "No commit message entered. Cancelled."
    exit
}

git add .

if (-not (git diff --cached --quiet)) {
    git commit -m "$Message"

    if ($LASTEXITCODE -eq 0) {
        git push
    }
}
else {
    Write-Host "No changes to commit."
}

git status