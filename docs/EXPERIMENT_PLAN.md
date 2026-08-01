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
   auktorisation. Key-only SSH är nu verifierat som generell administrativ
   transport, men den rörelseaktiverade hostadaptern och dess avgränsade
   forced-command-yta är ännu inte implementerade. Supervisorn äger motorerna
   exklusivt och verkställer heartbeat, touchstopp, stallstopp, timeout och
   lokala hårdgränser oberoende av LLM och nätverkslatens.

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
Den konkreta, rörelsefria ordningen för adapterinventering, ConnMan,
värdnyckelverifiering, länktest och återställning finns i
[`EV3_WIFI.md`](EV3_WIFI.md).

Status: Wi-Fi-onboarding, key-only SSH, enhetsidentitet över USB/Wi-Fi och
motorfri persistent sensortransport verifierades `2026-07-30`. Grinden för
säker rörelse över en bruten länk är inte passerad.

Verifierat:

- USB-adaptern identifierades som AR9271; `ath9k_htc`, nödvändig firmware,
  Wi-Fi-interfacet och ConnMan var redo.
- SSH accepterade endast publik nyckel i provet.
- Samma EV3-identitet verifierades först över USB och därefter över Wi-Fi,
  utan att nätverksnamn, adresser eller enhetsidentifierare lagras i denna
  rapport.
- En persistent SSH-process kunde bära upprepade IR- och touchläsningar utan
  ny anslutning för varje observation.
- Ett kontrollerat länkbortfall upptäcktes av hosttransporten och ConnMan
  återanslöt automatiskt.
- Efter en fysisk omstart återanslöt ConnMan automatiskt. Key-only SSH,
  sexfilers runtime-preflight och tre nya IR-läsningar passerade även sedan
  mini-USB-kabeln hade kopplats ur; Macens normala standardrutt var oförändrad.

Återstår:

- separat privilegiebegränsad/forced-command-yta för robottransporten,
- fysisk heartbeat- och lokal stoppverifiering vid länkbortfall,
- tappade och duplicerade kommandon,
- genomströmning för framtida ljuddata,
- Bluetooth-parning, PAN och RFCOMM om de spåren fortfarande är relevanta.

### EXP-F1B-WIFI-001 – motorfri Wi-Fi- och persistent sensortransport

- Datum: `2026-07-30`.
- Adapterns AR9271-chip, `ath9k_htc`, firmwareladdning och ConnMan verifierades
  före anslutning.
- Key-only SSH passerade. EV3:ans identitet matchade mellan USB- och
  Wi-Fi-vägen; faktiska SSID, IP-/MAC-adresser, machine-id och
  nyckelfingeravtryck ingår inte i evidensen.
- En kall full inventering överskred klientens tidigare deadline på `20 s`.
  Klienten observerade därför inget färdigt resultat och det är okänt om
  fjärrprocessen senare slutförde inventeringen. En omedelbar varm
  återkörning slutfördes på `15.721 s`.
- IR: den kalla begäran tog `13.307 s` och hela kalla sessionen `14.138 s`.
  Tio efterföljande varma läsningar gav min `70 ms`, median `82 ms` och
  p95/max `96 ms`. Värdet var `55 → 55`.
- Touch: den kalla begäran tog `16.995 s` och hela kalla sessionen
  `17.244 s`. Tre efterföljande varma läsningar gav min `73 ms`, median
  `86 ms` och p95/max `88 ms`. Värdet var `0 → 0`.
- Vid ett kontrollerat länkbortfall rapporterade hosten
  `PeripheralSSHTimeoutError` efter `3.005 s`. ConnMan hade återanslutit
  automatiskt vid den tredje kontrollen i en serie med `3 s` mellan
  kontrollerna.
- En efterföljande fysisk omstart verifierade ConnMans auto-connect. Efter att
  mini-USB kopplats ur saknades USB-interfacet, Macens standardrutt var
  oförändrad och strict key-only SSH fungerade fortfarande. Periferiprofilens
  runtime-preflight matchade `6/6` filer. Tre helt kabelbefriade varma
  IR-läsningar gav min `70 ms`, median `88 ms` och p95/max `91 ms`; värdet var
  `57 → 57`.
- Hela experimentet var motorfritt. De konstanta IR- och touchvärdena visar
  transportkontinuitet, inte att en fysisk stimulus upptäcktes.
- Timeout- och återanslutningsresultatet är inte evidens för motorstopp,
  heartbeat, bromssträcka eller säker fysisk exekvering vid länkbortfall.

Grind: länkbortfall stoppar lokalt, gamla kommandon återspelas inte och varje
kommando har ett unikt ID. Experimentet ovan passerar transportdelen men inte
denna rörelsegrind.

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

### EXP-F2-DAEMON-PREFLIGHT-002 – fysisk polltiming och fail-closed

- Datum: `2026-07-29`.
- Transport: USB-SSH över brickans link-local-adress; Macens standardrutt
  låg samtidigt kvar på sin vanliga Wi-Fi-anslutning.
- Batteri före proven: `7,476466 V`; efter första fail-closed-omgången:
  `7,421066 V`.
- Första foreground-försöket startade supervisorn i `DISARMED` men räknade
  reader-/writer-trådarnas cirka `33 ms` bootstrap mot en tillåten
  pollförsening på `20 ms`. Resultatet blev
  `startup_complete → fault_latched → supervisor_closed`, med
  `poll_deadline_missed`, verifierat stopp och noll motorstarter.
- Åtgärd: poll-epoken flyttades till efter att båda I/O-trådarna startat.
  Första-tick- och steady-state-grindarna behölls oförändrade. Ett nytt
  regressionstest injicerar `17 + 16 ms` trådstart och verifierar
  `CLOSED`, `fault=null` och noll `run-timed`.
- Den ändrade daemonen passerade `87` supervisortester, `25`
  transporttester, Python 3.5-grammatikkontroll och därefter hela sviten på
  `793` tester.
- Distribuerad daemon-SHA-256:
  `7744563d03a900eadb2a41e32f6b48fd9ceee41f717c2981950c39a7b5be6f83`.
  Föregående fil sparades lokalt på brickan, inte i repot, som
  `ev3/supervisor_daemon.py.before-poll-epoch-fix-20260729`.
- Andra foreground-försöket passerade trådbootstrap men missade en riktig
  poll-deadline med `220 ms`. Supervisorn latschade åter fault, stoppade och
  stängde i `CLOSED`; hosten skickade inget motion request.
- Ett separat rörelsefritt timingprov körde därefter tolv direkta
  `poll_once` i `DISARMED`. Tiderna var
  `[206,198,207,197,216,196,204,199,203,199,205,199] ms`, alltså
  `196–216 ms` med median `201 ms`. Alla tolv gav `fault=null`; slutläget
  var `CLOSED` med `audit_complete=true`.
- Efter varje misslyckad foreground-körning bekräftade ett separat
  `robot_cli.py stop` tre inaktiva, stabila motorer, tomma fault tokens och
  `stop_confirmed=true`. Encoderpositionerna för A/B/C förblev `0°`.

Slutsats: startup-felet är korrigerat utan att försvaga deadlinegrinden, men
den nuvarande Python/sysfs-pollvägen kan inte uppfylla konfigurationens
`20 ms`. Foreground-preflight och all fysisk motorstyrning förblir därför
blockerade. Nästa steg är att profilera och optimera den fysiska pollvägen
eller införa en separat, explicit och mätt kontrollprofil; gränsen ska inte
höjas enbart för att göra testet grönt.

## Fas 3 – Robot-API på värddatorn

Status: en första transportoberoende, typad slice är implementerad mot den
deterministiska simulatorn. Den exponerar endast den explicit allowlistade
propellerarmen. En separat concurrent-slice orkestrerar nu bounded
speech-callback och en semantisk, pausgrindad propellerreaktion bredvid
simulatornavigationen. Generella RobotAPI-verktyg för drivning och tal, varje
fysisk adapter och faktisk samtidig EV3-TTS återstår.

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

Detta fysiska shadow-resultat är separat från concurrent-simulatorn. Ingen
modellgenererad expression-text från den nya runtime-vägen har ännu skickats
till EV3-högtalaren.

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

`Talk/click-to-talk → lokal STT → synligt transkript → agent → robotverktyg → EV3-TTS`

Den nu implementerade slicen slutar vid det versionskontrollerade
agentformuläret. Ett verkligt Mac-mikrofonprov återstår, och robotverktyg,
färsk fysisk observation samt EV3-TTS i steg 3–6 nedan är framtida fysisk
acceptance.

Första versionens regler:

- en explicit Talk-knapp med automatisk tystnadsstopp i stället för ständig
  avlyssning,
- STT-körning bakom ett transportoberoende gränssnitt så motor eller modell
  kan bytas utan att agenten ändras,
- färdigt transkript levereras med hostägd timestamp och TTL till samma
  agentformulär som text; eventloggen behåller endast bounded metadata och
  aldrig transkript, råljud eller ljudhash,
- låg konfidens eller tvetydig instruktion leder till en kontrollfråga och
  aldrig till rörelse,
- samtalssvar får talas automatiskt men rörelse går alltid genom samma
  deterministiska robot-API och säkerhetspolicy som textkommandon,
- fysisk nödstopp får aldrig vara beroende av STT eller LLM,
- mikrofonen spärras eller ekoreduceras under EV3-TTS så att roboten inte
  transkriberar sitt eget tal och börjar prata med sig själv,
- varje inspelning och modellbegäran har timeout, avbrytning och episodbudget;
  en redan påbörjad lokal provideroperation får avslutas men dess sena resultat
  måste kastas efter Cancel eller shutdown.

Första acceptanstest:

1. Användaren trycker på Talk och säger "Vad ser du framför dig?"
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
  konsolens åtkomstnyckel, tokeniserad bootstrap-/assetväg, autentiserade
  läs-API:n,
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

Negativtest täcker bland annat DNS-rebinding-form, fel Origin/åtkomstnyckel,
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

### EXP-F4-I18N-002 – explicit svenska och engelska utan språkheuristik

Hypotes: gränssnitt och modellrespons kan byta språk reproducerbart utan att
hosten försöker klassificera användarens naturliga språk och utan att
språkbyte påverkar konversationens tekniska state.

Implementation:

- kompletta, nyckelmatchade svenska och engelska kataloger för statisk och
  dynamisk UI-copy, inklusive ARIA, placeholders, fel och tomlägen,
- standardsbaserat locale-val via `Intl.Locale`, lokalt sparat explicit val,
  browserlocale som fallback och locale-medveten datum-/nummerformatering,
- språkbyte utan omladdning, nätverksanrop, förlorat fokus, tappad
  textmarkering eller återställd konversationsscroll,
- typat och exakt allowlistat `response_locale` genom HTTP, idempotens,
  köad tur, kontext och LM Studio-systeminstruktion,
- hostskapad språkmetadata och `response_language_instruction` sist i
  modellkontexten samt på JSON-schemats user-facing textfält,
- auktoritativt svarsspråk för både `ANSWER` och `CLARIFY`, oberoende av
  språk i prompt, historik, evidens eller verktygsresultat,
- oförändrade tekniska värden såsom ID:n, hashes, modellnamn, eventtyper,
  verktygs-ID:n och rå JSON.

Kontrakts- och browserfria VM-tester verifierar katalogparitet, uppmärkning av
all statisk copy, locale-resolver, lagring, formatering, språkbyte utan
nätverk samt att servern avvisar saknad eller okänd `response_locale`.
Agenttester verifierar dessutom att valt språk följer exakt den köade turen
hela vägen till modellkontexten och ingår i idempotensidentiteten.

Resultat: svenska och engelska är godkända som förstaklasspråk för den lokala
dashboarden och agentens textsvar. Arkitekturen är förberedd för fler
kataloger, men andra språk är ännu inte innehålls- eller modelltestade. Hela
den hårdvarufria reposviten passerar `448 / 448`. Ingen EV3 kontaktades.

Liveprov 2026-07-27 med `google/gemma-4-26b-a4b`:

1. engelsk UI-copy, kuraterade experimentkort och registry-namn visades utan
   kvarvarande svensk presentationscopy,
2. språkbyte bevarade samtal, formulärstate och ett färdigt LM Studio-probe,
3. en första version lät vid ett tillfälle en engelsk prompt och engelsk
   historik vinna över svensk `response_locale`; detta fångades före commit,
4. efter att den hostskapade språkregeln placerats sist i kontexten passerade
   motexempel i båda riktningarna: svensk prompt som uttryckligen begärde
   svenska gav engelska i `en`, och engelsk prompt som uttryckligen begärde
   engelska gav svenska i `sv`,
5. samma resultat höll efter språkbyte mitt i en blandad konversation, och
   den engelska introduktionen blev naturlig user-facing copy: modellen
   beskrev sig som Robot LLM Labs lokala AI-assistent, inte som en intern
   planner.

Grind: godkänd för svensk och engelsk demo i den rörelsefria dashboarden.
Inte ett bevis på framtida flerspråkig STT, TTS, robotpersonlighet eller
fysisk instruktionsklassificering. Liveprovet är heller ingen matematisk
garanti för varje framtida modellutdata; en separat typad LLM-validator är
nästa förstärkning om svarsspråk ska ha en egen slutna-loop-grind. Före en
talande YouTube-demo ska engelskt STT och TTS verifieras som separata,
explicita locale-/voice-kontrakt.

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

### EXP-F5-NAV-SIM-002 – autonom waypoint och reaktiv hinderundvikning

Hypotes: den framtida navigationsarkitekturen kan provas end-to-end utan
EV3-batterier om perception, förslag, arbitrering, kort fysisk exekvering
och verifiering hålls som separata kontrakt, och om simulatorns
kollisionsfacit inte delas med plannern.

Implementation:

- strikt `robot-navigation-proposal/v1` där en planner endast får returnera
  `NEXT_SEGMENT`, `HOLD` eller `ABORT`; ett nästa segment är semantiskt
  `ADVANCE` eller `TURN`, aldrig råa hjul- eller motoranrop,
- hoststämplad `source_id`, source-sekvens, mottagningstid, TTL,
  authority-rank och priority; dessa fält kan inte sättas i modellens JSON,
- trådsäker bounded `ProposalInbox` som replaygrindar både proposal-ID och
  source-sekvens samt förbrukar hela batchen varje tick,
- immutable snapshot bundet till controllerinstans, goal epoch,
  plan revision, robot-state och world-model-version samt separata
  tidsstämplar för state och safety evidence,
- ensam `MotionSupervisor` som aldrig väntar på en producent, avgör exakt
  ett `STOP` eller en kort differential `DrivePulse` och stoppar vid stale,
  konflikt, touch, fault, aktiv motor eller okänd positiv clearance,
- syntetisk 2D-differential-drive-värld med pose, parvisa encoders,
  riktade metric-raycasts och ett separat swept-body collision oracle,
- bounded episodloop som verifierar hjulriktning, encoderpar,
  poseförflyttning, goal progress, budgetar, livelock och terminalt stopp.

Två lokala referensbeteenden används endast som simulatorbaslinjer:
`GoalSeekingBehavior` söker ett redan typat waypointmål och
`ObstacleAvoidanceBehavior` får högre hostauktoritet när ett hinder behöver
kringgås. De klassificerar inte text och innehåller inga språkregexp,
keywordlistor eller frasmenyer. En framtida asynkron Gemma-planner kan
publicera samma typ av förslag utan att MotionSupervisor ändras.

Deterministiskt demoscenario:

1. syntetisk robotradie `65 mm`, start `(150, 300)`, mål `(1000, 300)`,
2. cirkulärt hinder med centrum `(600, 300)` och radie `70 mm`,
3. högst `120 ms` per motorpuls,
4. waypoint nådd inom `30 mm` efter `98` ticks och `98` korta handlingar,
5. `147` förslag behandlade, `11 709 ms` total syntetisk motion,
6. noll collision-oracle-träffar och verifierat terminalt stopp.

Siffrorna ovan är reproducerbara simulatorutfall, inte mått på EV3RSTORM.
Konfigurationen ligger i `config/navigation_simulation.json` och är
uttryckligen `simulation_only`.

Negativ- och adversarialtest omfattar:

- duplicerad JSON-nyckel, extra auktoritetsfält och reverse som inte ingår i
  första kontraktet,
- proposal-ID-/source-sequence-replay, fel goal epoch, fel state-version,
  gammalt snapshot, gammal safety-observation, framtids-/TTL-gräns och
  förbrukade icke-vinnare,
- permutationsoberoende arbitrering där motstridiga likvärdiga förslag
  alltid stoppar,
- touch-latch och rearm först med nytt epoch och flera säkra snapshots,
- fysisk IR-reflektion `52` utan närträff som fortfarande inte får bli
  positiv metric clearance,
- tunnlingsförsök mot ett tunt hinder, återanvänt motorbeslut, encoderstall,
  falsk arbitersträng utan privat one-shot-auktorisation, gammal planrevision,
  world-model-byte efter auktorisation, trasig stoppobservation,
  två samtidiga auktoriserade pulser från samma snapshot, nytt goal epoch
  med nya producentinstanser, bounded replayfönster, asynkrona förslag som
  försöker kringgå proposal-budgeten, nekad rörelsebudget med
  `MotionAuthority(max_pending=1)` över upprepade episoder, safety-fault vid
  målet, tom producer-loop och exakta episodbudgetar,
- cold import som verifierar att navigationen inte laddar RobotAPI eller
  supervisortransport.

Efter slicen passerar hela den hårdvarufria reposviten `429 / 429`.

Grind: godkänd som simulator-first bevis för parallella förslagsproducenter,
serialiserad motion och sluten waypointnavigation. Inte godkänd för fysisk
drivning, fysisk centimeteravståndsmätning, robust okänd terräng, SLAM eller
A*. Fysisk readiness är false eftersom linjär kalibrering, verifierad
turn-kalibrering, stopplatens, bromssträcka och ett persistent
flerpulsprotokoll saknas. Ingen EV3 kontaktades.

### EXP-F5-CONCURRENT-SIM-003 – parallell navigation, uttryck, tal och pausgrindad propeller

Hypotes: långsam semantisk planering och taluppspelning kan isoleras från
navigationens tick, medan en armreaktion fortfarande kan serialiseras med
hjulen genom en explicit stoppgrind.

Implementation och acceptans:

- `ConcurrentBehaviorRuntime` kräver uttryckligen
  `DifferentialDriveSimulator`, `MotionSupervisor` och en bounded
  `ProposalInbox`; det finns ingen fysisk adapter i processen.
- Goal seeking och obstacle avoidance kör i oberoende bounded
  latest-snapshot-workers. Motionsticken väntar inte in alla workers och
  `MotionSupervisor` producerar fortfarande exakt ett hjulbeslut per tick.
- En interaction-reducer skapar stabila obstruction epochs och
  hostattribuerad evidens. Expression-resultat binds till robot,
  controllerinstans, mål, planrevision, world model, response locale,
  obstruction epoch och evidence-ID samt får en hostägd TTL.
- Reducern skapar dessutom en separat hostägd talkontext-generation.
  Identifierade objekt behåller samma talkontext över kort sensor-occlusion
  och nya fysiska obstruction epochs, medan andra objekt och oidentifierade
  nya hinder byter talkontext.
- Hosten härleder ett unikt expression-`proposal_id` från varje exakt
  snapshot, låser structured-output-schemat till detta konstanta värde och
  förbrukar ID:t exakt en gång per episod. Modellen kan därför inte svälta
  senare giltiga events genom att återanvända ett generiskt ID. Replay nekas
  och auditloggas. Tal kan accepteras efter ett nytt obstruction epoch endast
  om hostens talkontext fortfarande bevisar samma identifierade objekt och
  övriga robot-, controller-, mål-, plan-, world-, locale- och TTL-bindningar
  fortfarande gäller. Propellergesten återanvänds aldrig på detta sätt.
- Expression-anrop har både en total episodbudget och en cooldown per stabilt
  objekt-ID; oidentifierade hinder delar en konservativ unknown-nyckel.
  Upprepade återträffar på samma låda auditloggas men skapar inte en
  modell-/talspamloop, medan ett nytt objekt-ID är omedelbart valbart.
- LM Studio-adaptern använder strikt structured output och får endast svara
  `EXPRESS`, `HOLD` eller `ABORT`. `EXPRESS` innehåller en replik och antingen
  ingen gest eller exakt `PROPELLER_WAVE`; modellen kan inte ange motorport,
  hastighet, varaktighet, TTL, priority, authority eller source.
- Tal har en egen bounded och kooperativt cancellable worker. Ett fortfarande
  giltigt hinder-event får kommenteras medan navigationen fortsätter;
  blockerad eller felande planner/talcallback stoppar inte senare
  motionstick.
- Propeller-workern kräver däremot att samma hinder fortfarande är aktuellt.
  Den begär navigationspaus, väntar på stopped-boundary-kvittens, revaliderar
  evidens och TTL, kör fasta hostägda alternerande segment med cooldown-,
  antal- och tidsbudget och släpper sedan navigationen. Tester förbjuder
  wheel/arm-overlap.
- Köoverflow, malformad eller gammal modelloutput, callbackfel och
  cancellation ger typade metrics/auditevents. Cancellation signalerar
  aktiva callbacks kooperativt och navigationen gör fortfarande ett
  verifierat terminalt STOP. En host-watchdog begränsar den exklusiva
  armpausen och aborterar episoden om callbacken inte släpper, men fysisk
  fail-stop kräver fortfarande en lokal motortimeout i den framtida adaptern.
- `forward_object_id` får endast komma från simulatorns metric-raycast.
  Fysisk IR-reflektion kan varken identifiera ett objekt eller bevisa fri väg.

Resultat: godkänd som simulatorbevis för bounded parallell
förslagsproduktion, expression-resonemang och tal ovanpå serialiserad fysisk
avsikt. Själva sensorobservationen och interaction-reducern kör fortfarande
synkront på navigationens tråd; generell parallell perception är målarkitektur,
inte ett verifierat resultat här. Speech kan överlappa hjulnavigation;
propellern pausar avsiktligt hjulen. Testcallbacks är inte EV3-TTS eller en
fysisk armmotor, och slicen bevisar inte stopplatens, bromssträcka, fysisk
objektdetektion, kameraidentitet eller multi-controller-samordning. LM Studio-
adaptern och runtimen är kontraktstestade tillsammans via rå strukturerad
output.

Liveprov mot lokala `google/gemma-4-26b-a4b`:

- första concurrent-försöket fick ett LM Studio HTTP-fel; felet isolerades
  till expression-workern medan navigationen ändå nådde målet och stoppade
  verifierat,
- ett direkt efterföljande structured-output-prov gav en giltig svensk
  speech-only-proposal på cirka `2,94 s`,
- nästa kompletta concurrent-episod accepterade en svensk speech-only-
  expression efter cirka `3,7 s`; en navigationstick inträffade mellan
  `speech_started` och `speech_completed`,
- efter att `proposal_id` gjordes hostägt svarade Gemma giltigt mot det
  låsta schemat, men resultatet hann passera in i ett nytt obstruction epoch
  och släpptes därför som stale; navigationen fortsatte och stoppade
  verifierat,
- traceanalys visade att samma `demo-box` lämnade och återkom i den momentana
  sensorstrålen tre gånger medan roboten svängde; TTL hade inte löpt ut,
- efter införandet av separat talkontext accepterade motsvarande `50 ms`-
  scenario den svenska Gemma-repliken, gav noll stale expression- och
  speech-drops, överlappade en senare navigationstick och nådde målet efter
  `98` handlingar med verifierat terminalstopp,
- ett synkroniserat regressionstest visar samtidigt att en propellergest från
  den äldre snapshoten nekas efter epochskiftet: pause/STOP-kvittens och
  revalidering sker, men inga armsegment körs,
- upprepade engelska liveförsök isolerade ett separat kontraktsfel: ett
  instrumenterat Gemma-svar innehöll en replik på `170` tecken trots
  JSON-schemats `maxLength: 160`, varpå hostens strikta decoder nekade
  resultatet och navigationen fortsatte säkert,
- modellkontraktet gjordes därefter explicit kortare med högst `120` tecken
  både i prompt och schema. Nya svenska och engelska `50 ms`-försök gav
  giltiga egna repliker, tal/navigation-overlap, noll planner- och stale-fel
  samt verifierat terminalstopp,
- navigationen nådde waypointen efter `98` korta handlingar, terminalt stopp
  verifierades, inga workers levde kvar och modellen begärde ingen
  propellergest,
- speech-callbacken var fortfarande virtuell. Provet aktiverade varken
  högtalare, arm, EV3 eller annan fysisk hårdvara.

### EXP-F5-MISSION-SIM-004 – versionsbunden flerstegsplan

Hypotes: roboten kan exekvera en agentiskt användbar plan med flera delmål
utan att ge planbyggaren motorbehörighet eller låta ett delmål
förhandsauktorisera senare fysisk motion.

Implementation och acceptans:

- strikt `robot-navigation-mission-plan/v1` med 1–8 typade waypointben,
  unika leg-ID:n och exakt bindning till robot, controllerinstans,
  state-version, world-model-version och planrevision,
- varje ben får ett nytt monotont goal epoch och kör samma befintliga
  `NavigationEpisode` samt samma ensamma `MotionSupervisor`,
- missionens globala tick-, tids-, proposal-, replan-, action- och
  motionbudget klipps in i varje ben och kan inte återställas mellan ben,
- nästa ben får inte starta förrän föregående waypoint har nåtts och ett
  terminalt STOP har verifierats,
- ett misslyckat ben, cancellation eller budgetstopp förhindrar alla senare
  ben,
- en ändrad world-model-version gör återstående plan stale vid stoppgränsen
  och ger ett nytt versionsbundet STOP,
- ett världsbyte mellan observation och dispatch kasserar den gamla pulsen,
  gör en ny observation och förbrukar befintlig replanbudget; upprepade
  invalidationer slutar bounded med verifierat STOP,
- efter pausgrindad propellergest krävs en ny STOP/observationsgräns innan
  senare DRIVE.

Det deterministiska trebenstestet når samtliga waypointmål utan kollision och
verifierar ett terminalt STOP för varje ben. Negativtest täcker strikt JSON,
duplicerade ben, stale aktivering, världsbyte, stall, global actionbudget,
pre-cancellation och single-use-runner. Hela den hårdvarufria reposviten
passerar nu `574 / 574`.

Grind: godkänd som simulatorbevis för planexekvering, delmålsverifiering och
bounded failure propagation. Gemma bygger ännu inte godtyckliga
flerbensmissioner, missionsben kör ännu inte `ConcurrentBehaviorRuntime`, och
ingen fysisk adapter eller hårdvara aktiverades.

### EXP-F5-IDLE-SIM-005 – självvalda uppgifter och intresseklassificering

Hypotes: när inget användarmål finns kan en lokal modell välja ett eget
begränsat undersökningsmål från typade observationer utan att få koordinater,
motorbehörighet eller möjlighet att fördröja användarpreemption.

Implementation och acceptans:

- idle måste uttryckligen aktiveras och får endast starta med en exklusiv
  hostägd `IDLE_EXPLORATION`-lease,
- en väntande användare reserverar `USER_PENDING`, blockerar ny idle,
  cancellerar aktiv lease och får inte ett nytt mål förrän terminalt STOP
  verifierats,
- en separat state/world-versionerad observation beskriver exakt
  rangevärde och simulatoriskt objekt-ID från samma stoppade pose; hosten
  avgör inte språkligt vad som är intressant,
- hosten genererar endast geometriskt genomförbara lokala kandidater och
  håller koordinaterna i ett privat register,
- modellens strikta output är endast `SELECT | HOLD | ABORT`; `SELECT` låses
  i schemat till ett av de erbjudna opaka kandidat-ID:na,
- modellen får inte returnera waypoint, heading, path, goal epoch,
  motoruppgifter, speed, duration, tool call, TTL, source, priority eller
  authority,
- modellresultatet revalideras mot lease-generation, proposal-ID,
  kandidatset, exklusiv deadline, state-version, world-model-version,
  observationens robot/controller/frame samt host-receive/TTL och ett nytt
  säkert stoppat snapshot,
- en förändring under modellsvaret kasserar svaret och konsumerar
  replanbudget; en förändring under första DRIVE gör missionen stale,
  verifierar STOP och kräver en ny lease/epoch,
- modellvalet körs single-flight och väntan avbryts direkt av user
  reservation eller deadline; ett sent/hängt jobb får aldrig fler
  parallella selectortrådar,
- cancellation före dispatch återkallar pulsen; en reservation inne i den
  sista `plant.apply`-skarven tillåter högst den enda redan dispatchade,
  tidsbegränsade pulsen, därefter inga fler DRIVE och verifierat STOP,
- slutförda waypointceller sparas först efter nått mål och verifierat STOP,
  medan misslyckade attempts når ett deterministiskt retrytak om inte exakt
  ny observation återöppnar cellen,
- planneranrop, tasks, stale-replans, hosttid, actions och total motion har
  både kumulativa sessionsbudgetar och en beständig duty-cycle över nya
  scheduleranrop; re-arm kräver avstängd och säkert stoppad idle och hålls
  atomisk mot samtidiga idle-enable/user-claim via en authority-guard,
- fysisk `IR-PROX` får varken skapa metric observation eller positiv
  idle-kandidat.

Deterministiska simulatorresultat:

- tre självvalda uppgifter slutfördes med `34` korta DRIVE-pulser,
  noll kollisioner och terminalt STOP efter varje uppgift,
- ett range-change-scenario flyttade samma syntetiska låda mellan två
  stoppgränser; den andra observationen var exakt `207 → 357 mm` och
  kandidaterna bar `INVESTIGATE_OBSERVATION`,
- race-testet blockerar användaraktivering medan idle fortfarande äger
  målet, avbryter väntan även på ett selectorjobb som inte återkommer,
  kasserar det sena svaret, verifierar STOP och tilldelar därefter användaren
  ett strikt nyare goal epoch,
- negativa test täcker `HOLD`, tom kandidatmeny, malformed/okänt ID,
  state- och world-staleness, deadline exakt på gränsen och efter selection,
  världsbyte före första DRIVE, bounded apply-race, attempts/retrytak,
  user-reservation-cancel, minnescommit samt sessions- och duty-cyclebudgetar,
- ett långt clear-world-regressionstest slutför `12 / 12` självvalda mål
  inklusive gränsnära kandidater utan kollision eller fastnad
  obstacle-avoidance; ett separat CLI-stresstest slutförde `20 / 20` mål med
  `269` korta pulser och noll kollisioner.

Liveprov med lokala `google/gemma-4-26b-a4b`:

- vanlig idle-session: `2 / 2` modellvalda uppgifter, `27` actions,
  noll kollisioner och verifierat STOP,
- range-change-session: `2 / 2` modellvalda uppgifter, den andra vald som
  `INVESTIGATE_OBSERVATION`, totalt `22` actions, noll kollisioner och
  verifierat STOP,
- modellen valde endast schemaerbjudna opaka ID:n; hosten löste därefter
  koordinaterna och den befintliga deterministiska navigationen utförde
  uppgiften.

Hela den hårdvarufria reposviten passerar `656 / 656`.

Grind: godkänd som simulatorbevis för självvalda, bounded högre mål,
modellbaserad intresseklassificering och säker målpreemption. Inte godkänd för
fysisk idle-körning. Tal, gest, dashboardmål, kamera, mikrofon och flera
controllers är ännu inte integrerade med denna lease.

### EXP-F5-SPATIAL-SIM-006 – kontinuerligt spatialt världsminne

Hypotes: robotens versionsbundna navigationsobservationer kan kontinuerligt
bygga en osäker, begränsad omgivningskarta utan att raycasting,
occupancy-fusion, objektsamling eller GUI-serialisering flyttas in i
motionsticken, och utan att fysisk IR felaktigt behandlas som centimeter.

Implementation och acceptans:

- `NavigationSnapshot` publiceras genom en O(1), bounded, drop-oldest
  `offer_nowait`-relay till exakt en separat mapping-worker;
- navigation, mission, idle och concurrent runtime isolerar sinkfel och
  publicerar dessutom sin slutliga verifierade STOP-snapshot;
- workern äger en trådsäker LRU-grid med fast celltak, egen monoton
  kartrevision och immutable snapshots;
- robot/controller/frame, state-version, world-model-version och tidsordning
  valideras före varje atomisk ingest; stale eller duplicerad evidens muterar
  inte kartan;
- simulatorns tre strålar vid `0°` och `±45°` fusionerar fri respektive
  upptagen evidens genom ett explicit `UNKNOWN`-intervall; korrelerade
  strålar reduceras till en uppdatering per cell och en occupied endpoint
  dominerar samtidig free-evidens i samma grova cell;
- samma sensor-timestamp återfusioneras inte från ett nyare robot-state,
  medan en högre `world_model_version` atomiskt nollställer den gamla
  evidensgenerationen och får ta in sitt första prov även vid samma
  simulatortid;
- simulatorprovenance heter
  `SIMULATION_CONFIGURATION_SPACE:*`, eftersom avståndet gäller
  robotradie-inflated kollisionsgeometri och inte exakt fysisk objektyta;
- fysisk `IR-PROX` skapar endast bounded, lågkonfidens,
  `PROVISIONAL_QUALITATIVE` evidens utan millimeter, metrisk endpoint,
  objektnamn eller positiv clear-path;
- connected occupied cells skapar persistenta, opaka `UNKNOWN`-hypoteser med
  bounds, centroid, ålder, confidence och evidenslinje som senare
  kamera-/LLM-klassificering kan referera till;
- relaydrop eller mapperfel ger synlig `degraded` status men kan inte stoppa
  eller auktorisera motion;
- dashboarden får endast en read-only snapshot-provider och exponerar en
  autentiserad `GET /api/v1/map` med strikt JSON; den slutligt kodade
  HTTP-kroppen, inklusive `map`-envelopen, har en exakt 4 MiB-responsgräns;
- GUI:t visar fri/upptagen/osäker grid, robotpose, färska strålar,
  objekthypoteser, källa, provenance, ålder och simulator/provisional-status
  på svenska och engelska.

Det deterministiska end-to-end-scenariot kör den befintliga
hinder-navigationen och bygger kartan asynkront. Resultat:

- `98` korta DRIVE-handlingar, noll kollisioner och verifierat terminalt
  STOP;
- `100` snapshots applicerade utan relaydrop eller mapperfel;
- `193` gridceller kvar efter fusion och `9` opaka connected-component-
  hypoteser, varav minst en bär det betrodda simulator-ID:t `demo-box`;
- den verkliga runtime-snapshoten passerar dashboardens JavaScript-
  normalisering med robotpose, tre sensorstrålar, celler och hypoteser;
- en startad loopback-dashboard gav autentiserad HTTP `200` för samma
  faktiska karta medan `physical_control_enabled` förblev false;
- hela den hårdvarufria reposviten passerar `scripts/quality_check.sh`.

Negativtest täcker stale/duplicerade snapshots, robot/controller-mismatch,
återanvänt sensorprov, generationsbyte, överlappande rays, motstridig evidens
som går via `UNKNOWN`, fysisk IR-isolering, bounded
cell-/evidensminne, blocked mapper med queue overflow, drop-oldest-semantik,
mapperfel, flush/close, invalid eller sen publication, providerfel,
oversize/malformad dashboarddata, route-auth och sinkfel som inte får påverka
mission eller terminalt stopp.

Grind: godkänd som simulatorbevis för asynkron kontinuerlig mapping och
read-only visualisering. Inte godkänd för fysisk metrisk mapping, SLAM,
loop closure, kartbaserad path planning, frontier exploration eller semantisk
objektklassificering. Kartan är ännu observationsdata och återkopplas inte
till `MotionSupervisor`. Ingen EV3 eller annan fysisk hårdvara kontaktades.

### EXP-F5B-PHYSICAL-NAV-EVIDENCE-001 – kropp, scanminne och erfarenhet

- Datum: `2026-08-01`.
- Status: implementerad och hårdvarufritt testad; fysisk låd-acceptans
  återstår.

Hypotes: den fysiska agenten kan undvika oförändrade försök och behålla ett
allt rikare men bounded beslutsunderlag om hosten publicerar faktisk
kroppsgeometri, posebunden scan-evidens och strukturerade action/result-fakta
utan att själv välja modellens rutt.

Implementerad slice:

- EV3RSTORM-profilen har en asymmetrisk, provisorisk rektangel runt
  differential-drive-origo. Separata extents modellerar att högerarmen sticker
  ut längre än vänstersidan; måtten är ännu inte fysiskt uppmätta.
- Varje erbjuden rörelse och aktiv scan kontrolleras mot hela den
  interpolerade svepta kroppen och aktuella provisoriska hinder. Samma kontroll
  upprepas före dispatch. Hosten tar bort omöjliga handlingar men rangordnar
  eller väljer ingen ersättare.
- Återställda scans sparar faktisk scanpose, kartbasis, requested/actual
  body-relative bearing, blocked/clear och unilateral/bilateral gränsevidens.
  Retention prioriterar olika evidenssignaturer framför duplicerade retries
  inom gränsen 16 poster per hinder och 64 per karta.
- Varje detaljpost som lämnar 16/64-retentionen räknas med typad orsak på
  hinder och kumulativt på kartan. Kartnivån inkluderar även scanposter som
  försvinner med en utkastad hinderhypotes och överlever save/load.
- Blockerade historiska strålar ger konservativa angular supports med
  provisorisk fast offset; de är inte metriska objektkonturer. Indexet behåller
  högst 512 supportfakta per hinder och 4 096 per karta, oberoende av att äldre
  scannars detaljprojektion roterar. En full bilateral all-clear contestar
  hypotesen utan att radera historiken, och senare typad blocked-evidens kan
  återaktivera kollisionshypotesen.
- Hinderkartan behåller högst 64 hypoteser och navigation-memory-filen högst
  2 MiB. Kapacitetsbortfall räknas och publiceras med typad orsak i både
  planner- och dashboardkontext; det får inte ske som en osynlig FIFO-förlust.
- Persistensformat `v2` läser den tidigare fysiska `v1`-artefakten och migrerar
  den i minnet. Nästa save skriver `v2`; en kodrollback till en läsare som bara
  kan `v1` kräver backup av den äldre filen eller en uttrycklig minnesreset.
- En ruttcommitment kräver kompletterande positiv och negativ gräns från exakt
  robotens aktuella verifierade scanpose. Evidens från en äldre pose får stanna
  i kollisionsminnet men får inte återanvändas som aktuell vänster/höger-gräns.
- Ett episodlokalt `NavigationExperienceLedger` behåller strukturerade
  försök/resultat, högst 64 detaljposter/`64 KiB` lokalt och ett LRU-index över
  43 200 tidigare `(typed attempt, evidence basis)`-par. Gemmas separata
  projektion behåller högst `24 KiB` detalj med exakta totalsummor, senaste
  typade utfall, bortfallsräknare och digest. Indexgränsen motsvarar tre
  möjliga handlingar per tillåten runtime-turn under en hel episod. Det
  skiljer första försök,
  oförändrad repetition och retry efter en verifierad förändring i beslutsfakta.
- Hela planner-användarkontexten har `56 KiB` mål och `64 KiB` hårt tak. En
  konservativ 32k-admission räknar även systemprompt, dynamiskt output-schema,
  wrapperreserv, headroom och 520 outputtokens. Maxfixturen med 64 hinder,
  64 scans och full ledger mäts både med hostestimat och Gemma-4-tokenizer.
- Evidensbasen ignorerar state-version, tidsstämplar, liten rå IR-jitter,
  duplicerade scan-ID:n och icke-drivande armmotorer. Den använder verifierad
  pose, drivencoders, beslutssensorer, hinder och substantiell scan-evidens.
- Dashboardens read-only fysiska lager visar den konfigurerade kroppskonturen,
  historiska scanposer och faktiska blocked/clear-vinklar. Ingen stråle får
  uppfunnen fysisk längd.

Hårdvarufria scenarier täcker bland annat att högerarmen ger annan
svepgeometri än vänstersidan, att en geometriskt omöjlig modellhandling inte
erbjuds, att äldre scanperspektiv nekas som route evidence, att duplicerade
scans inte tränger undan unik gränsevidens och att en action/basis-cykel känns
igen efter att den detaljerade ledgerhistoriken roterat. De täcker även
`v1 → v2`-migrering, att scan-ID:n är persistensbundna och control-safe, att
maximala runtime-genererade detalj- och supportmängder ryms under 2 MiB samt att
hypotesbortfall vid karttaket överlever save/load och syns i båda kontexterna.

Fysisk evidensgräns: den föregående lådkörningen observerade att högerarmen
tog i hindret och motiverade den nya profilen, men hela implementationen ovan
körde inte då. Den äldre scanartefakten
`data/EXP-EV3-LIVE-SCAN-20260801-001.json` innehåller rådata endast för ett
stoppat försök med `2,5°` tolerans. Senare toleransprov saknar ännu en
korrelerad incheckad råartefakt. Nästa grind är därför samma autonoma
lådscenario med den nya koden, följt av publicerad observation, scan,
action/result-ledger och terminalt utfall. Fysiskt propelleruttryck ingår
inte i denna slice.

## Fas 6 – Parallella snurror

Simulator-slicen bevisar nu en avgränsad del av
schemaläggningsmönstret: synkrona sensorobservationer matar parallella workers
för navigation proposals, expression planning och tal samt en separat
asynkron spatial-map-worker, medan ett enda motionplan serialiserar hjulen och
pausgrindar propellern. Den fysiska målarkitekturen är fortfarande att även
sensorpollning, generell perception, planering och validering arbetar
parallellt, en host-side decision arbiter serialiserar semantiska
beslutsförslag och varje lokal LEGO-supervisor ensam serialiserar och
övervakar redan auktoriserade motorprimitiver.

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
