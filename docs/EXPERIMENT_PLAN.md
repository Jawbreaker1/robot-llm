# Kontrollerad experimentplan

## Invariant

LLM:n får aldrig direkt tillgång till SSH, godtycklig kodkörning eller
motorns sysfs-filer. Den får endast begära verktyg från ett strikt robot-API.
En lokal EV3-supervisor behåller exklusivt motorägarskap.

## Ansvarsgränser för språk och rörelse

1. **Semantiskt agentplan:** LLM:n får originalinstruktionen,
   verktygskontraktet och versionsmärkt strukturerad kontext. Den
   klassificerar avsikt, löser referenser och producerar ett typat
   beslutsförslag.
2. **Deterministiskt auktorisationsplan på hosten:** validerar förslaget mot
   schema, allowlist, aktuellt tillstånd, färska observationer, behörighet,
   konflikter och episodbudget. Lagret får inte tolka originalspråk eller
   använda regexp, substrings eller keywords som alternativ klassificerare.
3. **Lokal EV3-supervisor:** tar emot redan typade, tidsbegränsade
   motorprimitiver. I målarkitekturen når de den först efter host-policyns
   auktorisation; transport och autentisering är ännu inte implementerade.
   Supervisorn äger motorerna exklusivt och verkställer heartbeat, touchstopp,
   stallstopp, timeout och lokala hårdgränser oberoende av LLM och
   nätverkslatens.

Ogiltig modell-JSON, okänd avsikt, gammal kontext eller olöst referens leder
till `reject` eller `clarify`, aldrig till en heuristiskt gissad handling.
`MotionCommand` är en intern fysisk primitive och får inte vara det kontrakt
som visas direkt för språkmodellen.

## Experimentprotokoll

Varje fysisk körning dokumenterar:

1. experiment-ID och hypotes,
2. fysisk uppställning och abortmetod,
3. kodversion och konfiguration,
4. maximalt tillåten hastighet, tid och rörelse,
5. tillstånd före körningen,
6. begärt och godkänt kommando,
7. faktiskt resultat och tillstånd efter körningen,
8. slutsats och exakt nästa förändring.

En fas passerar först när dess lyckade fall är reproducerbara och dess
viktigaste felfall har testats avsiktligt.

## Fas 1 – Hårdvarukarakterisering

Status: pågår.

Klart:

- SD-boot, USB CDC, SSH och Python.
- Högtalare, ton och lokal svensk eSpeak.
- Passiv motor- och sensorinventering.
- Tidsbegränsad rörelse av motor B och C med encoderverifiering.
- Motor C verifierad som höger drivben med positiv riktning framåt.
- Motor B verifierad som vänster drivben med positiv riktning framåt.
- Motor A verifierad som propellerarm med encoderåterkoppling.
- Parade B/C-pulser verifierade med gemensamt lås, lokal timeout, jämna
  encoderdelta och visuell observation.
- Högersväng på stället verifierad med encoderpostcondition och visuell
  observation.
- Touchsensorn verifierad med tryck- och släppövergångar.
- IR-sensorn verifierad med varierande närhetsvärden.
- Färgsensorn verifierad i reflektionsläge.
- Svensk lokal eSpeak-TTS verifierad över EV3-högtalaren.
- Första rörelsefria kedjan `IR-observation → kort kommentar → TTS`
  verifierad manuellt.
- Samlad icke-interaktiv shadow-CLI verifierad över USB-SSH med publik
  Mac-nyckel, tre IR-läsningar, lokal Gemma och deterministisk TTS.
- Lokal motorfri IR-grind verifierad i två dynamiska, röstguidade cykler vid
  `20 Hz`, inklusive ett sparat rådatareplikat.

Återstår:

- rotationsriktning för motor A:s propeller,
- motorburen IR-inflygning först efter lokal supervisor, med uppmätt faktisk
  stopplatens och bromssträcka,
- färgklassificering mot kontrollerade färgprover,
- upprepade stopp- och positionstester.

Grind: alla anslutna enheter kan läsas eller aktiveras manuellt med
reproducerbara, begränsade testfall.

### EXP-F1-TTS-001 – avståndskommentar utan rörelse

- IR-sensorn rapporterade `67 %` i `IR-PROX`; värdet behandlades som relativ
  närhet och inte centimeter.
- En deterministiskt vald svensk kommentar spelades med eSpeak i `125 wpm`.
- Resultat: TTS-pipelinen rapporterade `completed`; inga motorer aktiverades.

### EXP-F1-TTS-002 – härdad agenttextväg

- Ett separat `read-sensor` läste IR-sensorn i verifierat `IR-PROX`-läge och
  gav ett tidsstämplat relativt värde på `82 %`.
- Text skickades via stdin till ett fast `speak-stdin`-kommando, inte som
  shellkod eller ett eSpeak-argument.
- Lokal svensk eSpeak kördes i `125 wpm`, amplitud `140`, och rapporterade
  `completed` efter `9411 ms`.
- Lokala gränser omfattar 160 tecken, tillåten röst, talhastighet, amplitud,
  exklusivt ljudlås och en hård deadline på `20000 ms`.
- Samtliga `38` lokala tester passerar, inklusive timeout, processdödning,
  låsfrigöring, ogiltiga talvärden och fel sensorläge.

### EXP-F1-IR-CAL-001 – statisk IR-kalibrering

Robot och motorer var stilla. Avstånd uppskattades från IR-sensorns framsida
och varje statiskt test använde 20 behållna värden efter stabilisering.

| Mål | Avstånd | Median | Spann |
|---|---:|---:|---:|
| ljust, brett | 15 cm | `7` | `7–8` |
| mörkt, brett | 15 cm | `13` | `13` |
| smal låda, cirka 10 cm bred | 15 cm | `28` | `28` |
| ljust, brett | 30 cm | `25` | `24–27` |
| mörkt, brett | 30 cm | `28` | `27–28` |
| ljust, brett | 50 cm | `45` | `44–45` |
| stort skåp | cirka 100 cm | `50` | `50–51` |

Ett manuellt sidosvep med den smala lådan på cirka 15 cm gav bakgrund `52`
och två tydliga passager med hinderintervall `26–34`. Ingen av passagerna
nådde den först föreslagna gränsen `≤16`; den gränsen hade därför missat ett
verkligt närhinder.

Provisorisk tolkning:

- `≤16`: stark retur, inte ett säkert avståndsmått,
- `≤35`: kandidat för konservativ närhindergrind,
- `≥40`: grinden får börja frigöras efter stabila upprepningar,
- `>47`: fjärrsvag eller oklar retur, aldrig bevis på fri väg.

Grinden använder median över tre värden, två samstämmiga närträffar för
inträde och tre högre värden för frigivning. Startup och ogiltiga värden är
fail-closed. En mycket stark råträff `≤16` stoppar omedelbart.

Status: godkänd som provisorisk klassificering och som underlag för dynamiska
tester, men inte ännu som ensam eller live-ansluten kollisionssäkring. Touch,
korta rörelsepulser, lokal timeout och en framtida EV3-supervisor kvarstår.
Samtliga `82` lokala tester passerar, inklusive replay av lådsvepet,
den dynamiska IR-proben, LM Studio-protokollfel och den rörelsefria
shadow-kedjan.

### EXP-F1-IR-DYN-001/002 – dynamisk evidensgrind utan motorer

Robotkroppen stod stilla och `stop_all` bekräftade A, B och C som stoppade.
Lådan fördes för hand mot och bort från IR-sensorn. EV3 samplade lokalt vid
`20 Hz`; deterministiska röstprompter markerade faserna och varken Gemma
eller motorstyrning ingick i beslutet.

Första fullständiga cykeln:

- `444` råvärden, spann `52 → 33 → 41`,
- första råvärdet `≤35` vid `23415 ms`,
- medianfiltret nådde `≤35` vid `23465 ms`,
- hinderstatus efter andra samstämmiga filtrerade träffen vid `23515 ms`,
- beslutslatens `100 ms` från första råträffen och `50 ms` från första
  filtrerade tröskelkorsningen,
- första råvärdet `≥40` under returfasen vid `32761 ms`; ett efterföljande
  `39` bröt råsekvensen,
- första stabila filtrerade `≥40` vid `32860 ms` och frigivning efter tredje
  filtrerade träffen vid `32961 ms`,
- frigivningslatens `101 ms` från första filtrerade tröskelkorsningen och
  `200 ms` från den tidigaste råträffen.

Det reproducerbara replikatet kördes därefter med
`ev3/ir_gate_probe.py`, som läser exakt samma policy från konfigurationen:

- `277` råvärden, spann `42 → 31 → 40`,
- faktisk period `47–53 ms`, medel `50 ms`,
- första råvärdet `≤35` vid `15651 ms`, första filtrerade `≤35` vid
  `15701 ms` och hinderstatus vid `15751 ms`,
- första råvärdet `≥40` vid `24751 ms`, första filtrerade `≥40` vid
  `24801 ms` och frigivning vid `24901 ms`,
- resultat `status: completed`.

Full råserie och policy finns i
`docs/data/EXP-F1-IR-DYN-002.json`.

Slutsats: medianfönster `3`, två inträdesträffar `≤35` och tre
frigivningsträffar `≥40` fungerar reproducerbart som en lokal
evidensklassificerare vid `20 Hz`. Detta mäter inte motorstopplatens,
bromssträcka, sant avstånd eller fri väg. Nästa motorburna test kräver först
en lokal EV3-supervisor som kan stoppa motorerna i samma pollingloop.

### EXP-F1-SHADOW-001 – Gemma-kandidat med deterministisk talgrind

- Native LM Studio `POST /api/v1/chat` verifierades med
  `google/gemma-4-26b-a4b`, `reasoning: "off"`, `store: false`,
  `integrations: []` och utan modellverktyg.
- Ett separat test med zonen `near_return` och relativvärdet `28` gav en
  vanlig kort svensk kandidat på cirka `0,19 s` och exakt `0`
  resonemangstoken. Det äldre OpenAI-kompatibla försöket gav tom `content`
  eftersom hela budgeten förbrukades av dolt resonemang.
- I det första fysiska failover-provet läste EV3 ett färskt IR-värde på `58`.
  Det klassificerades som `far_or_no_clear_return`, vilket inte betyder fri
  väg. LM Studio-tjänsten var då otillgänglig.
- Ingen modelltext skickades vidare. EV3 sade den deterministiska frasen
  "Jag får ingen tydlig närträff framför mig." och den lokala TTS-pipelinen
  rapporterade `completed` efter `4723 ms`.
- Därefter installerades endast Macens befintliga publika Ed25519-nyckel för
  användaren `robot`; `BatchMode=yes` verifierades utan lösenordsprompt.
- Den fulla CLI-cykeln läste `[58, 58, 58]`, klassificerade medianen `58` som
  `far_or_no_clear_return` och fick en formellt giltig Gemma-kandidat efter
  `417 ms`: "Irriterande, IR-zonen är helt otydlig och jag vägrar att
  analysera den mer."
- Kandidaten loggades men talades inte. EV3 sade åter den deterministiska
  frasen och rapporterade `completed` efter `4589 ms`. Hela CLI-körningen
  rapporterade `status: completed`.
- Inga motorer aktiverades. Testerna verifierar dessutom att även en
  strukturellt giltig hallucination, exempelvis en påhittad hund och ett
  avstånd, bara loggas och aldrig når TTS i shadow-läget.

Status: modellprotokollet, failoverprincipen och den samlade fysiska
shadow-cykeln är godkända. Modelltext förblir auditdata; eventuell befordran
till TTS kräver en separat semantisk evalueringsgrind.

### EXP-F1-PAIR-001 – första parade drivpulsen

- Hypotes: samma positiva robotrelativa hastighet får B och C att röra sig
  framåt med jämförbara encoderdelta.
- Abortmetod: EV3-knapp eller lyft, lokal `run-timed` på båda motorerna och
  explicit dubbelstopp efter körningen.
- Preflight: båda drivmotorerna stilla, touch `0`, batteri `8,0164 V`.
- Rörelsefritt negativtest: höger `251 dps` avvisades mot gränsen `250 dps`.
- Godkänt kommando: vänster `+100 dps`, höger `+100 dps`, `300 ms`.
- Resultat vänster B: position `254 → 280`, delta `+26°`.
- Resultat höger C: position `231 → 258`, delta `+27°`.
- Uppmätt förskjutning mellan de sekventiella startskrivningarna: `7 ms`.
- Efterläge: kommandot rapporterade `completed`; därefter kördes `stop_all`
  och A, B samt C rapporterades stoppade.
- Slutsats: den begränsade parade HAL-transaktionen fungerar på fysisk EV3.
  Encoders verifierar motorrotation; faktisk förflyttning verifierades i
  uppföljningsexperimentet nedan.

### EXP-F1-PAIR-002 – synlig framåtrörelse

- Syfte: upprepa den parade transaktionen med en tydligt observerbar men fortsatt
  begränsad rörelse.
- Preflight och efterläge: `stop_all` kördes före och efter pulsen.
- Godkänt kommando: vänster `+200 dps`, höger `+200 dps`, `800 ms`.
- Resultat vänster B: position `307 → 482`, delta `+175°`.
- Resultat höger C: position `286 → 461`, delta `+175°`.
- Uppmätt förskjutning mellan startskrivningarna: `5 ms`.
- Visuell observation: rörelsen bedömdes som mycket bra; ingen avvikelse
  rapporterades.
- Slutsats: rak framåtdrift är en reproducerbar fysisk primitive vid denna
  hastighet och tidsgräns. Detta ändrar inte kravet på supervisor, heartbeat
  och kollisionsstopp före autonom körning.


### EXP-F1-TURN-001 – högersväng och verifierad effekt

- Första försök: vänster `+150 dps`, höger `−150 dps`, `600 ms`.
- Första resultat: B `+86°`, C `0°`. Den äldre HAL-versionen rapporterade
  `completed` trots utebliven högerrörelse.
- Isoleringsprov: B backade `−168°`; C gav först `0°` och därefter `−164°`
  vid samma `−200 dps` i `800 ms`. Bakåtriktningen fungerar men den uteblivna
  C-rörelsen var intermittent.
- Åtgärd: HAL fick en encoderpostcondition. Varje motor måste röra sig minst
  `3°` i den fysiskt begärda riktningen; annars misslyckas hela handlingen.
  Meningslöst små rörelsebegäranden avvisas före motorskrivning.
- Feltester: nollrörelse, fel encoderriktning och ensidig parad rörelse
  avvisas. Totalt `26` lokala tester passerar.
- Slutligt kommando: vänster `+200 dps`, höger `−200 dps`, `800 ms`.
- Slutligt resultat vänster B: position `402 → 574`, delta `+172°`.
- Slutligt resultat höger C: position `297 → 127`, delta `−170°`.
- Uppmätt förskjutning mellan startskrivningarna: `6 ms`.
- Båda encoderpostconditionerna passerade och högersvängen bekräftades
  visuellt.
- Slutsats: `turn_right` kan byggas ovanpå den parade primitiven. Verifieringen
  är ännu en postcondition efter rörelsen; live stall- och kollisionsstopp
  tillhör EV3-supervisorn.

### EXP-F1-TURN-002 – grov 90-graderskalibrering

- Plan: fyra vänstersegment med vänster `−200 dps`, höger `+200 dps`,
  `800 ms` per segment.
- Exekveringsregel: nästa segment startades endast om båda encoderpostconditionerna
  i föregående segment passerade.
- Segmentens B-delta: `−171°`, `−173°`, `−169°`, `−172°`; totalt `−685°`.
- Segmentens C-delta: `+167°`, `+173°`, `+168°`, `+171°`; totalt `+679°`.
- Samtliga åtta motorpostconditioner passerade.
- Visuell observation: roboten vred sig ungefär `90°` runt sin egen axel.
- Första kalibreringsestimat: medelvärdet `682°` encoderrotation motsvarar
  cirka `90°` kroppsvridning, eller ungefär `7,58` encodergrader per grad.
- Begränsning: estimatet är underlags-, batteri- och slirningsberoende och har
  bara en visuell observation. Ett framtida `turn_approx_degrees` måste därför
  uttrycka osäkerheten och verifierar motorrotation, inte absolut orientering.

## Fas 1B – Trådlös transport

USB behålls som återställningsväg under hela utvärderingen.

Spår:

- Bluetooth-parning och tjänsteinventering.
- Bluetooth PAN om värddatorn erbjuder nätverksprofilen.
- Bluetooth RFCOMM för ett litet seriellt kommandoformat.
- USB Wi-Fi-dongel för normal IP, SSH och senare robot-API.

Mätningar:

- rundturstid och variation,
- faktisk genomströmning,
- återanslutning,
- tappade och duplicerade kommandon,
- beteende när länken bryts mitt under en begäran.

Grind: länkbortfall stoppar lokalt, gamla kommandon återspelas inte och varje
kommando har ett unikt ID.

## Fas 2 – EV3-supervisor

Status: språkblind kärna implementerad och verifierad mot fake sysfs/fake
clock. Den rörelsefria preflighten har även passerat på den fysiska EV3:an.
En foreground-process, ett strikt nodmärkt protokoll och en Mac-klient har
dessutom verifierats rörelsefritt mot falsk sysfs med riktiga subprocesser
och OS-pipes. Kärnan är fortfarande inte godkänd för autonom fysisk körning.

Implementerat:

- livslångt exklusivt motorlås,
- serverutfärdad session och strikt stigande sekvens-ID,
- lokalt tidsbegränsade kommandon,
- hårda argumentgränser,
- prioriterat, sessionsoberoende stopp,
- monotona tidsstämplar,
- heartbeat och lokal timeout,
- touchbaserad stop-input,
- encoderbaserad stall- och riktningskontroll,
- absolut-deadline-baserad pollingloop med latenhetsgrind,
- latched fault utan automatisk återstart,
- stopprekondition, upprepade stoppförsök och verifiering av inaktivt,
  stabilt motorläge,
- explicit `brake` före `stop`; okända state-tokens nekas och `holding`
  behandlas som aktivt motorläge,
- begränsad, icke-blockerande auditbuffer under supervisorns exekvering,
- append-only JSONL-flush efter den rörelsefria preflightens shutdown,
- rörelsefri preflight-CLI,
- foreground-daemon över stdin/stdout avsedd för en autentiserad SSH-kanal,
- exakt JSONL-schema; duplicerade JSON-nycklar och icke-finita tal nekas,
- separat `robot_id`, `controller_id` och processunik
  `controller_instance_id`,
- lokalt mottagningsstämplad `queue_ttl_ms`, recheck precis före varje
  motorstart och avbrottsgrind före varje motor i en parad start,
- unika request-ID:n, strikt controller-routing och prioriterat nödstopp,
- bounded input/output-köer och I/O utanför safety-/dispatchtråden,
- `describe` med effektiva capabilities, processgränser och rörelsebudget,
- publik daemon-entrypoint som alltid annonserar och verkställer
  `motion_enabled=false` och saknar motion-enable-flagga,
- rörelsefri Mac-preflight som aldrig konstruerar eller skickar
  `drive_timed`,
- explicit `OPEN → CLOSING/POISONED → CLOSED` i hosttransporten,
- kanalpoison före nästa write vid timeout, partiell skrivning,
  korrelationsfel, dubbla/oväntade svar, EOF eller asynkront readerfel.

Vid denna supervisorslice omfattade den dåvarande fulla sviten `246` tester.
Dess supervisor- och transportfall täcker bland annat ägarskap, replay,
feladresserat shutdown, heartbeat exakt vid `500 ms`, touch före och under
rörelse, ensidig stall, fel
encoderriktning, partiell motorstart, oväntad rörelse i tredje motorn,
auditfel före och efter motorstart, långsam I/O, avbrott mellan parade
motorstarter, klockregression, missad poll-deadline före och efter poll,
stopfel, shutdown med bibehållet motorlås, deadline/cancel mellan parade
motorstarter och att preflight aldrig skriver `run-timed` eller rapporterar
framgång efter en misslyckad shutdown/audit. Riktiga subprocessprov omfattar
ren handshake, stdin-EOF, `SIGTERM`, trasig JSON-frame och fylld stdout-pipe.
Raceprov täcker dessutom request under close, wait/poison mot pågående
writer, samtidiga stop-/motionförslag och att känd streamdesynk aldrig följs
av en ny write.

Återstår före en första motorpuls:

- fysisk rörelsefri daemon-preflight över den betrodda USB-SSH-länken,
- separat SSH-nyckel/forced command före motion-enabled användning,
- lock-retaining fail-stop/retry om shutdown inte kan verifieras; nuvarande
  publika process kan inte aktivera motion och är därför endast en
  preflight-yta,
- avsiktligt dödad riktig SSH-klient och fysisk signal-/auditkontroll,
- fysisk polling- och stopplatens,
- fysisk bromssträcka och låg-hastighetskalibrering av stallgränser,
- verifiering vid sensorbortfall, motorjam, USB-bortfall och supervisor-krasch.

Grind: avsiktligt dödad klient, bruten länk, felaktiga argument och programfel
leder till ett uppmätt och säkert stopp.

### Backendprotokollet är inte agentens verktyg

`claim`, `heartbeat`, `arm`, `drive_timed`, `release`, `stop` och `shutdown`
är en intern exekveringsyta mellan hostadapter och fysisk controller. Gemma
ska aldrig se eller anropa den. Det framtida semantiska robot-API:t översätter
typade `drive`/`turn`-förslag till dessa primitiv först efter separat
host-auktorisation.

`queue_ttl_ms` börjar när en komplett frame mottagits på EV3 och jämförs
endast med EV3:ans monotona klocka. Det är inte en end-to-end-timestamp från
Macen. Startgrinden kontrollerar deadlinen igen direkt före varje
`run-timed`.

### EXP-F2-SUP-PREFLIGHT-001 – fysisk rörelsefri supervisor

- Datum: `2026-07-25`.
- Precondition: A, B och C rapporterade tomt motor-state; touch var `0`.
  Batteriet rapporterade `6,9414 V`, vilket accepteras för ett rörelsefritt
  prov men inte för nästa motorfas.
- Supervisor-SHA-256:
  `1dca0e018753f78ace19a0363941225d71f097b4b84782da94322f2466539888`.
- Preflight-CLI-SHA-256:
  `f4c57d902e1e1c9e1b1d27e3900ddaa95415aab7d5e40b4df420837f35900a11`.
- EV3 körde Python `3.5.3`; Macens betrodda Ed25519-värdnyckel matchade den
  anslutna USB-adressen före överföring.
- Kommando: `python3 ev3/supervisor_cli.py preflight`.
- Resultat: `status=completed`, `motor_start_commands=0`, stabil touch
  `3/3`, `DISARMED` före shutdown och `CLOSED` med
  `audit_complete=true`.
- Både startup och shutdown skrev `brake` följt av `stop` till A/B/C och
  verifierade två stabila, inaktiva snapshots utan fel eller fault-tokens.
- Encoderpositionerna var oförändrade mellan auditposterna:
  A `770°`, C `1151°`, B `−434°`.
- Efterkontroll: samtliga motor-state var tomma, `stop_action` var `brake`
  och ett separat icke-blockerande försök kunde ta och frigöra motorlåset.
- Audit: `/tmp/robot-llm-supervisor-audit.jsonl` innehöll
  `startup_complete` följt av `supervisor_closed`.
- Återställningskopia av föregående konfiguration:
  `/home/robot/robot-llm/config/ev3rstorm.before-supervisor.json`.

Slutsats: den rörelsefria start-/stopplivscykeln fungerar på den riktiga
EV3:an och rapporterar inte framgång utan stabil touch, verifierat motorläge,
terminal audit och stängd supervisor. Experimentet innehöll ingen armering,
inget `run-timed` och ingen rörelse. Det godkänner därför inte fysisk
superviserad motorstyrning eller autonomi.

## Fas 3 – Robot-API på värddatorn

Status: en första transportoberoende, typad slice är implementerad mot den
deterministiska simulatorn. Den exponerar endast den explicit allowlistade
propellerarmen; drivning, tal, semantiska kompositverktyg och fysisk adapter
återstår.

Det fulla framtida API:t byggs ovanpå simulator eller fysisk EV3:

- `get_robot_state`
- `read_sensors`
- `drive`
- `turn`
- `move_arm`
- `wave_arm`
- `speak`
- `play_tone`
- `stop`

Grind: alla verktyg och alla nekande fall fungerar manuellt utan LLM.

### EXP-F3-API-SIM-001 – typad armgräns utan hårdvara

- Immutable observationer binds till robot, controller, processinstans,
  hostklocka, state-version och mottagningstid.
- Motion kräver exakt capability, färsk snapshotreferens, unik hostmyntad
  action/segment-identitet och kort deadline.
- Allowlisten innehåller endast `arm`; två drivmotorer och en olistad framtida
  hjälpmotor nekas.
- Check, dispatch, state-version och stop serialiseras under en single-writer-
  lock. Två parallella förslag baserade på samma snapshot kan därför inte
  båda exekveras, och inget validerat förslag kan starta efter ett returnerat
  stopkvitto.
- Nödstopp är instansbundet men får upprepas; motionsegment har konservativ
  `at_most_once`-semantik och ett tappat kvitto återvinns ännu inte.
- Simulatorn annonserar `accelerated_synchronous`: encodereffekten appliceras
  omedelbart. Provet säger inget om live-avbrott, heartbeat eller fysisk
  stopplatens.

Resultat: godkänd hårdvarufri kontraktsgrind för armprimitiven. Inte godkänd
som fysiskt RobotAPI.

### Första perceptions-/språkloopen

Den första loopen är medvetet rörelsefri:

`IR → medianfilter → deterministisk zon → Gemma-kandidat i shadow → validering/logg → deterministisk TTS`

Gemma får endast zonen och det relativa mätvärdet och får inte välja verktyg.
I första grinden talas aldrig Gemmas kandidat; ett ogiltigt, sent eller
uteblivet modellsvar registreras och samma deterministiska fras används.
Maxlängd, timeout, ljudlås och pratbudget gäller oberoende av modellen. Först
efter en separat semantisk evalueringsgrind kan validerad modelltext
övervägas för TTS, fortfarande utan motorbehörighet.

## Fas 4 – Chatt och kontext

LM Studio får endast se verktygskontraktet och ett litet strukturerat minne:

- aktuellt mål,
- senaste begärda handling,
- senaste godkända handling,
- senaste faktiskt genomförda handling,
- senaste observation,
- aktiva fel.

Acceptanstest:

1. "Vinka med höger arm."
2. "Två gånger till."
3. "Gör samma sak med vänster."
4. "Lite långsammare."
5. "Sluta."

Varje mål testas dessutom med svenska parafraser, minst ett annat språk,
ellipser och pronomen, negation, citerat språk och ett simulerat STT-fel.
Tvetydiga referenser måste ge `clarify`. Okända fält, gammal kontext,
ogiltiga verktyg och schemafel måste alltid nekas utan regexp- eller
keyword-fallback.

### Röstgränssnitt via värddatorns mikrofon

Den första mikrofonen sitter i Macen och är en inmatningskanal till samma
chattkontext:

`push-to-talk → lokal STT → synligt transkript → agent → robotverktyg → EV3-TTS`

Första versionens regler:

- push-to-talk i stället för ständig avlyssning,
- STT-körning bakom ett transportoberoende gränssnitt så motor eller modell
  kan bytas utan att agenten ändras,
- transkript, konfidens och tidsstämpel loggas som en observation,
- låg konfidens eller tvetydig instruktion leder till en kontrollfråga och
  aldrig till rörelse,
- samtalssvar får talas automatiskt men rörelse går alltid genom samma
  deterministiska robot-API och säkerhetspolicy som textkommandon,
- fysisk nödstopp får aldrig vara beroende av STT eller LLM,
- mikrofonen spärras eller ekoreduceras under EV3-TTS så att roboten inte
  transkriberar sitt eget tal och börjar prata med sig själv,
- varje inspelning och modellbegäran har timeout, avbrytning och episodbudget.

Första acceptanstest:

1. Användaren håller push-to-talk och säger "Vad ser du framför dig?"
2. Transkriptet visas i applikationen.
3. Agenten begär en färsk IR-observation.
4. Gemma formulerar en kort kommentar i vald personlighet.
5. Validerad text spelas över EV3-högtalaren.
6. Robotens eget tal skapar inte ett nytt STT-meddelande.

Wake word, kontinuerlig avlyssning och en robotmonterad mikrofon behandlas som
senare, separata experiment.

### EXP-F4-RESEARCH-001 – lokal Gemma med färsk, citerad väderevidens

Hypotes: samma lokala modell som senare ska tolka robotmål kan redan nu välja
ett externt informationsverktyg, observera ett typat resultat och formulera
ett källbundet svar i en sluten loop utan språkheuristiker och utan någon
fysisk capability.

Säkerhetsgräns:

- research-modulerna importerar inte RobotAPI, SSH, TTS, supervisor eller
  motorprimitiver,
- plannern får endast returnera `CALL_TOOL`, `ANSWER`, `CLARIFY` eller
  `ABORT` enligt ett dynamiskt strikt JSON-schema,
- enda registrerade verktyget är `weather.current`,
- nättransporten använder två fasta Open-Meteo HTTPS-origins, inga proxies
  och inga redirects; godtyckliga URL:er accepteras inte,
- toolanrop, proposal-ID:n, host-ID:n, kontextversioner, plannerlatens,
  episodtid och antal omplaneringar har separata grindar,
- svar med `require_evidence=true` måste citera ett exakt, färskt
  hostmyntat evidence-ID,
- providertext behandlas som passiv `untrusted_external_data`.

Hårdvarufria negativtest omfattar duplicerade JSON-nycklar, okända och
fysiska verktygsnamn, prompt injection i providerdata, gamla
kontextversioner, proposal-replay, host-ID-kollisioner, fel request/plats,
fel geocoding-query, fel koordinater, gammal observationstid, utgången
evidens, falska citationer, överskridna bytegränser, redirects, proxyfri
transport, MIME-fel och verkligt vidarebefordrad planner-timeout.

Liveprov 2026-07-26:

1. användarfråga: "Behöver jag paraply i Stockholm just nu?",
2. Gemma `google/gemma-4-26b-a4b` valde typat
   `weather.current(location_query="Stockholm")`,
3. Open-Meteos
   [geocoding-API](https://open-meteo.com/en/docs/geocoding-api) löste
   Stockholm och dess [forecast-API](https://open-meteo.com/en/docs)
   returnerade aktuella fält,
4. evidensen bands till requesten med provider-URL:er, hämtningstid, TTL,
   byteantal, SHA-256, attribution och
   [CC BY 4.0](https://open-meteo.com/en/license),
5. nästa Gemma-varv returnerade `ANSWER` med exakt det aktuella
   evidence-ID:t.

Resultat: `ANSWERED` efter `2` planner-varv, `1` toolanrop och `1`
evidensdriven omplanering. Providerns aktuella modellvärde hade
giltighetstiden `2026-07-26T21:45 UTC`; Open-Meteos `current` är
15-minuters modellbaserade data, inte en fysisk instrumentobservation, och
`precipitation` avser föregående 15-minutersintervall. Detta är ett enskilt
liveprov, inte ett benchmark eller en garanti om hela dagens väder. De fasta
gratis-endpoints som används är avsedda för icke-kommersiell, rate-limited
prototypanvändning utan upptidsgaranti enligt
[pricing](https://open-meteo.com/en/pricing) och
[terms](https://open-meteo.com/en/terms). EV3:an kontaktades inte och ingen
TTS eller motorväg fanns i processen. Vid denna researchgrind passerade den
dåvarande hårdvarufria sviten `305 / 305`; den aktuella totalsviten redovisas
i README och efterföljande experiment.

Grind: godkänd som första lokala, evidensbaserade agentloop och som
byggkloss för ett framtida chattgränssnitt. Inte godkänd som generell
webfetch. En sådan transport kräver först DNS-/IP-pinning, blockering av
lokala/särskilda nät, redirectvalidering per hopp samt hårda
MIME-/byte-/deadline-regler. Research-evidens får inte heller i framtiden
föra fysisk auktoritet.

### EXP-F4-DASHBOARD-001 – lokal Lab Console utan robotström

Hypotes: Macapplikationen kan redan utan EV3-batterier ge ett användbart
agentgränssnitt för dialog, följdfrågor, research, komponentstatus,
agentbudgetar och tekniska loggar utan att skapa en ny fysisk
exekveringsväg.

Implementation:

- fem responsiva ytor: Arbetsbänk, Kroppar, Händelser, Experiment och
  Inställningar,
- loopback-only standardbiblioteksserver med slumpad 256-bitars
  sessionstoken, tokeniserad bootstrap-/assetväg, autentiserade läs-API:n,
  strikt `Host`/`Origin`, CSP, begränsad HTTP-concurrency och exakt
  route-/assetlista,
- bounded kö med en researchworker; HTTP-status och eventpollning blockeras
  inte av en pågående modellförfrågan,
- revisionsmärkt, sessionsbunden settingssnapshot per köad tur,
- versionerad `typed_history` med endast tidigare synliga
  user/assistant-turer, utan textkonkatenering, regexp eller keyword-routing,
- multi-robot-registry med EV3RSTORM, framtida Robot Inventor/BOOST,
  kameror och mikrofoner; alla noder har `control_exposed=false`,
- teknisk ringbufferlogg med monotona sekvenser, cursor-gap och
  korrelations-ID:n; prompt, rå modelltext, evidence-URL och traceback
  utelämnas.

Liveprov 2026-07-27:

1. dashboarden startade på `127.0.0.1:8765` och returnerade strikt CSP,
2. LM Studio-proben hittade den konfigurerade
   `google/gemma-4-26b-a4b` som laddad,
3. en vanlig fråga gav `ANSWERED` på ett planner-varv utan toolanrop,
4. tvåturstestet `Mitt namn är Johan` → `Vad heter jag?` gav
   `Du heter Johan.` med fyra synliga meddelanden i `typed_history`,
5. researchfrågan om paraply i Stockholm gav `ANSWERED` efter två
   planner-varv, ett `weather.current`-anrop och ett citerat evidence-ID.

Negativtest täcker bland annat DNS-rebinding-form, fel Origin/sessionstoken,
dubblettnycklar, `NaN`, okänd MIME, för stor body, traversal, queue-full,
settings- och konversationsrace, idempotent retry, stale resultatidentitet,
rå prompt/exception i eventloggen samt frånvaro av `/move`, `/stop`, `/ssh`
och `/tts`.

Resultat: godkänd som lokal, rörelsefri agentarbetsbänk. Hela den
hårdvarufria sviten passerar `381 / 381`. EV3 kontaktades inte och servern
importerar inte RobotAPI, supervisortransport, TTS eller motorprimitiver.

Grind: godkänd för fortsatt dialog-, research- och GUI-utveckling utan
robotström. Inte godkänd som fysisk kontrollpanel. Settings lagras endast i
minnet i denna slice och återställs vid omstart.

## Fas 5 – Sluten målriktad loop

Loop:

`mål → observera → planera → ett steg → observera → verifiera → omplanera`

Varje episod har budget för tid, agentvarv, körsträcka, omplaneringar och
upprepade handlingar. Framgång måste verifieras med färska observationer.
Endast nästa fysiska steg auktoriseras; en hel plan får aldrig förhandsgodkänd
motorbehörighet.

Status: loopens första hårdvarufria vertikala slice är implementerad med ett
typat encoderpositionsmål och en scriptad planner. LM Studio, naturlig
språkklassificering och fysisk exekvering ingår ännu inte.

### EXP-F5-LOOP-SIM-001 – observera, agera, verifiera, omplanera

- Plannern får bara returnera exakt `ACT` eller `ABORT` enligt ett strikt
  JSON-schema; duplicerade nycklar, extra fält, boolska heltal och
  icke-finita värden nekas.
- Hostkoden kontrollerar mål-ID, state-version, motorroll, riktning,
  capability, action-TTL, total rörelsetid, global episoddeadline och
  omplaneringsbudget.
- Två begränsade armsteg når `100°`; varje steg verifieras mot kvitto,
  state-version och faktisk encoderprogress innan nästa planeringsvarv.
- Fel riktning, fel motor, för hög fart, gammalt state, replay, plannerfel,
  osäker touch och utebliven progress leder till stopp eller kontrollerad
  omplanering.
- Terminal framgång kräver färsk, säker observation efter exakt verifierat,
  instansbundet stopp. Stopprekonditionen provas två gånger; oväntade
  backendfel återkastas först efter best-effort-stop.
- Episoddeadlinen kontrolleras efter planner, före dispatch och efter
  exekvering, med reserverad rörelse- och stopptid.

Resultat: godkänd som simulatorbevis för den generiska beslutspipelinen. Nästa
separata grind är en LM Studio-planner mot samma kontrakt, fortfarande utan
fysisk motion.

## Fas 6 – Parallella snurror

Sensorpollning, perception, planering och validering får arbeta parallellt.
En host-side decision arbiter serialiserar semantiska beslutsförslag. Den
lokala EV3-supervisorn serialiserar och övervakar endast redan auktoriserade
motorprimitiver.

En logisk robot kan bestå av flera exekveringsnoder. Varje EV3-, Robot
Inventor- eller BOOST-controller får exakt en lokal motorägare. En gemensam
host-koordinator väljer samordnade handlingar men kan inte kringgå någon
lokal supervisor. Kameror och mikrofoner är separata perceptionsnoder som
publicerar tidsstämplade observationer och aldrig motorbeslut. Se
[`ARCHITECTURE.md`](ARCHITECTURE.md).

## Fas 7 – Kamera, mikrofon och aktiv perception

Kamera och mikrofon behandlas som tidsstämplade observationskällor. Ett
långsiktigt demonstrationstest är:

`hundskall → ljudriktning → visuell sökning → hund bekräftad → social respons`
