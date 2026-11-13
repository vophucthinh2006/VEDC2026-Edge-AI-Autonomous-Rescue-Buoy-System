# The boat's spoken messages (Rasp_Pi/sounds/*.wav, played by modules/announcer.py), made on a
# Windows PC with the vi-VN voice "Microsoft An" (Settings > Time & language > Speech > add Vietnamese).
#   powershell -ExecutionPolicy Bypass -File Rasp_Pi\tools\make_sounds.ps1 [-Rate 0.7]
# Rate is the voice's speaking rate, 1.0 normal; lower is slower. Letters outside ASCII are written
# <hhhh> (their code point) so this file stays ASCII: Windows PowerShell reads a file without BOM in
# the ANSI code page.
param([double]$Rate = 0.7)
$messages = [ordered]@{
    'rescue_boat'        = '<0110><00e2>y l<00e0> t<00e0>u c<1ee9>u h<1ed9>'           # "Day la tau cuu ho"
    'supplies_confirmed' = '<0110><00e3> x<00e1>c nh<1ead>n l<1ea5>y <0111><1ed3>'   # "Da xac nhan lay do"
}
$out = Join-Path (Split-Path -Parent $PSScriptRoot) 'sounds'
New-Item -ItemType Directory -Force -Path $out | Out-Null

Add-Type -AssemblyName System.Runtime.WindowsRuntime
$null = [Windows.Media.SpeechSynthesis.SpeechSynthesizer, Windows.Media.SpeechSynthesis, ContentType = WindowsRuntime]
$null = [Windows.Storage.Streams.DataReader, Windows.Storage.Streams, ContentType = WindowsRuntime]
$asTask = [System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object {
    $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1' } | Select-Object -First 1
function Await($op, [Type]$type) { $task = $asTask.MakeGenericMethod($type).Invoke($null, @($op)); $task.Wait(); $task.Result }
function Decode([string]$text) { [regex]::Replace($text, '<([0-9a-f]{4})>', { param($m) [string][char][Convert]::ToInt32($m.Groups[1].Value, 16) }) }

$synth = New-Object Windows.Media.SpeechSynthesis.SpeechSynthesizer
$voice = [Windows.Media.SpeechSynthesis.SpeechSynthesizer]::AllVoices | Where-Object { $_.Language -eq 'vi-VN' } | Select-Object -First 1
if (-not $voice) { throw 'no vi-VN voice installed' }
$synth.Voice = $voice
$synth.Options.SpeakingRate = $Rate
foreach ($name in $messages.Keys) {
    $stream = Await ($synth.SynthesizeTextToStreamAsync((Decode $messages[$name]))) ([Windows.Media.SpeechSynthesis.SpeechSynthesisStream])
    $reader = New-Object Windows.Storage.Streams.DataReader($stream.GetInputStreamAt(0))
    $null = Await ($reader.LoadAsync([uint32]$stream.Size)) ([uint32])
    $bytes = New-Object byte[] ($stream.Size)
    $reader.ReadBytes($bytes)
    $path = Join-Path $out "$name.wav"
    [System.IO.File]::WriteAllBytes($path, $bytes)
    '{0}: {1} bytes, rate {2}' -f $path, $bytes.Length, $Rate
}
