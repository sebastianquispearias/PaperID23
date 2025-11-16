param(
  [int[]] $Seeds = @(123,124,125,126,127,128),
  [int]   $Throttle = 6,              # máximo procesos en paralelo (tú tienes 12 lógicos)
  [string]$CondaEnv = "",             # opcional: por ejemplo "base"; dejar vacío si no usas conda
  [string]$Entry   = "experiments.py",
  [string]$ExtraArgs = "--no_vis" ,     # opcional: flags extra para acelerar o silenciar visualización
  [int]   $StaggerSec = 2
)

$ErrorActionPreference = "Stop"
$here = $PSScriptRoot
Set-Location $here

# Python a usar
if ($CondaEnv -ne "") {
  $python = "conda"
  $pyArgsPrefix = "run -n $CondaEnv python"
} else {
  $python = "python"
  $pyArgsPrefix = ""
}

# Carpeta de logs
$logDir = Join-Path $here "logs"
if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir | Out-Null }

Write-Host ("Lanzando {0} seeds (Throttle={1})..." -f $Seeds.Count, $Throttle)

# Cola y estructuras
$queue   = [System.Collections.Queue]::new()
$Seeds | ForEach-Object { $queue.Enqueue($_) }
$running = @{}   # seed -> Process
$done    = @{}   # seed -> exitCode

function Start-Seed([int]$seed) {
  $outLog = Join-Path $logDir ("seed_{0}.txt" -f $seed)
  $errLog = Join-Path $logDir ("seed_{0}.err.txt" -f $seed)

  # Lanzamos via cmd para poder definir variable de entorno EXP_SEEDS SOLO para este proceso
  if ($CondaEnv -ne "") {
    $file = "cmd.exe"
    $args = "/c set EXP_SEEDS=$seed && conda run -n $CondaEnv python $Entry $ExtraArgs"
  } else {
    $file = "cmd.exe"
    $args = "/c set EXP_SEEDS=$seed && python $Entry $ExtraArgs"
  }

  "== seed job started: $(Get-Date -Format s) ==" | Out-File -FilePath $outLog -Encoding utf8

  $p = Start-Process -FilePath $file `
                     -ArgumentList $args `
                     -RedirectStandardOutput $outLog `
                     -RedirectStandardError  $errLog `
                     -PassThru `
                     -WorkingDirectory $here
  return $p
}


# Bucle principal (scheduler simple)
while ($queue.Count -gt 0 -or $running.Count -gt 0) {

  # Llenar hasta el throttle
  while (($running.Count -lt $Throttle) -and ($queue.Count -gt 0)) {
    $seed = [int]$queue.Dequeue()
    $proc = Start-Seed $seed
    $running[$seed] = $proc
    Write-Host (">> running seed {0} (pid {1})" -f $seed, $proc.Id)
    Start-Sleep -Seconds $StaggerSec
  }

  Start-Sleep -Seconds 2

  # Recolectar finalizados
  foreach ($seed in @($running.Keys)) {
    $proc = $running[$seed]
    if ($proc.HasExited) {
      $code = $proc.ExitCode
      $done[$seed] = $code
      $running.Remove($seed)

      if ($code -eq 0) {
        Write-Host ("seed {0}: OK" -f $seed) -ForegroundColor Green
      } else {
        Write-Host ("seed {0}: FAIL({1})" -f $seed, $code) -ForegroundColor Red
      }
    }
  }
}

# Resumen
Write-Host "---- RESUMEN ----"
foreach ($s in $Seeds) {
  $code = -1
  if ($done.ContainsKey($s)) { $code = $done[$s] }
  if ($code -eq 0) {
    Write-Host ("seed {0}: OK" -f $s) -ForegroundColor Green
  } else {
    Write-Host ("seed {0}: FAIL({1})" -f $s, $code) -ForegroundColor Red
  }
}

Write-Host ("Logs en: {0}" -f $logDir)
Write-Host "Para ver un log en vivo:  Get-Content .\logs\seed_123.txt -Wait"
