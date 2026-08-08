# Lokal Mac-dashboard

Dashboarden är Robot LLM Labs lokala kontroll- och observationsyta. Den fungerar
även utan EV3-batterier och samlar lokal textdialog och röstinmatning, read-only
research, tekniska händelser, agentbudgetar och ett beskrivande register över
nuvarande och framtida robotkomponenter. Fysisk styrning är helt avstängd om
processen inte uttryckligen startats med en robotadapter.

## Starta

LM Studio är valfritt för att öppna gränssnittet, men krävs för att få svar
från Gemma.

```sh
scripts/start_lab_console.sh
```

Öppna `http://127.0.0.1:8765`; den lokala roten skickar webbläsaren vidare
till den aktuella liveadressen i formen
`http://127.0.0.1:8765/live/<access-key>/`. Åtkomstnyckeln i länken är en
lokal bearer-hemlighet: dela den inte. Redirecten innebär medvetet att andra
lokala processer kan upptäcka nyckeln; servern är därför fortsatt strikt
bunden till loopback. Startprofilen lagrar en
slumpad 256-bitars nyckel i en owner-only-fil, så samma bookmark fortsätter
peka på den aktuella konsolen efter en omstart. Äldre
`/session/<key>/`-länkar omdirigeras till den kanoniska liveadressen.

Servern binder alltid till den numeriska loopback-adressen `127.0.0.1`.
Porten kan ändras:

```sh
scripts/start_lab_console.sh --port 8877
```

## Fem skilda saker – ingen sessionslista

- **Livekonsolens åtkomstnyckel** låser upp den enda aktuella lokala
  konsolen. Den identifierar inte ett sparat körläge och är inget man
  "återupptar".
- **Workbench-konversationen** är dialogkontext för Gemma. En ny konversation
  påverkar inte robotens fysiska läge eller världsminne.
- **Den fysiska episoden** är ett målriktat robotförsök från start till
  terminalt tillstånd. Gränssnittet visar den pågående episoden, annars den
  senast avslutade tydligt märkt som historisk.
- **Experimenthistoriken** är read-only evidens från dokumenterade körningar.
  Den kan granskas men är inte en gammal fysisk robotinstans som kan startas
  igen.
- **Världs-/kartminnet** är separat host-lokal navigationsdata. Varje fysisk
  EV3-episod skapar automatiskt en ny lokal kartgeneration vid `(0, 0, 0)`.
  Observationer bevaras under hela episoden men återanvänds inte i nästa,
  eftersom EV3 saknar en absolut referens som kan upptäcka att roboten har
  flyttats. `--robot-memory-path` väljer episodens atomiska arbetsfil.
  Dashboardens heta events, snapshots och pose trail är processlokala.

För att först köra den riktiga 2D-simulatornavigationen och därefter visa dess
ackumulerade karta i dashboarden:

```sh
PYTHONPATH=src python3 -m robot_agent.dashboard_cli \
  --simulation-map-demo
```

Flaggan kontaktar ingen EV3. Med en fysisk `ev3rstorm-01`-profil ansluts i
stället en separat fysisk kartprovider; den visar "inga observationer" tills
runtime:n har tagit sin första verifierade EV3-snapshot. Utan simulator eller
fysisk profil visar kartytan ärligt att ingen kartprovider är ansluten. De två
kartkällorna får inte kombineras i samma dashboardprocess.

### Starta med lokal taligenkänning

Projektets normala röstprofil återanvänder en varm lokal `whisper.cpp`-tjänst
med `large-v3-turbo-q5_0`, som är kvalitetsstandarden för svensk och engelsk
mikrofoninmatning:

```sh
scripts/start_lab_console.sh
```

Den kanoniska runtime-standarden är det exakta LM Studio-ID:t
`google/gemma-4-26b-a4b-qat`. Startprofilen laddar eller byter inte modell;
det ID:t måste redan exponeras av avsedd LM Studio-server. Ett explicit
alternativ gäller konsekvent för både Workbench- och Robot-inställningen i den
processen:

```sh
scripts/start_lab_console.sh --model 'EXACT-MODEL-ID-FROM-LM-STUDIO'
```

Profilen ansluter som standard till `http://127.0.0.1:8178/v1`, använder den
separat validerade inferenssökvägen `/audio/transcriptions` och probar tjänsten
innan dashboarden rapporteras som startklar. På den primära utvecklingsdatorn
hålls tjänsten redan varm utanför dashboarden och ska inte startas en gång
till.

På en ny Mac installeras modellen och en kompatibel tjänst så här:

```sh
brew install whisper-cpp
sh scripts/download_whisper_model.sh large-v3-turbo-q5_0
whisper-server \
  --model models/ggml-large-v3-turbo-q5_0.bin \
  --host 127.0.0.1 \
  --port 8178 \
  --threads 8 \
  --language auto \
  --no-timestamps \
  --suppress-nst \
  --request-path /v1 \
  --inference-path /audio/transcriptions
```

Modellen ligger i den git-ignorerade katalogen `models/`. Nedladdningsscriptet
accepterar `large-v3-turbo-q5_0`, `small` och `base` och verifierar den
officiella filens SHA-256 innan den aktiveras. De två mindre modellerna är
endast explicita jämförelsealternativ.

Profilen kan ändras portabelt med `ROBOT_LLM_STT_URL`,
`ROBOT_LLM_STT_INFERENCE_PATH`, `ROBOT_LLM_STT_MODEL_ID` och
`ROBOT_LLM_PYTHON`. Vanliga CLI-argument skickas vidare oförändrade.
Om den externa tjänsten saknas kan dashboarden uttryckligen äga en modell
under sin egen livstid. Stoppa först en konkurrerande GPU-tjänst, eller välj
en isolerad CPU-körning:

```sh
ROBOT_LLM_STT_URL='' scripts/start_lab_console.sh \
  --stt-model models/ggml-large-v3-turbo-q5_0.bin \
  --stt-port 8180
```

Externa STT-adresser måste vara loopback. Bassökvägen måste vara antingen den
exakt tillåtna kompatibilitetssökvägen `/v1` eller ett långt opakt privat
segment. Inferenssökvägen valideras separat som en kanonisk relativ sökväg:
värdnamn, query, fragment, tomma segment, traversal och icke-ASCII-former
nekas. `/v1` är en kompatibilitetsregel, inte en autentiseringshemlighet.

## Språk

Språkväljaren i toppfältet byter hela dashboardens presentationsspråk utan
omladdning eller ny nätverksbegäran. Svenska och engelska är kompletta
kataloger i den första slicen. Det explicita valet sparas lokalt i browsern;
utan ett sparat val används browserns standardsbaserade locale och därefter
svenska som fallback.

Valet skickas även som det typade fältet `response_locale` på varje agenttur.
Fältet valideras genom en exakt allowlist och blir auktoritativt för
`ANSWER`- och `CLARIFY`-texten. Modellen ska alltså inte gissa svarsspråk från
användartext, historik, evidens eller verktygsresultat. Det gör språkbyte
explicit och reproducerbart utan regexp, nyckelord eller språkheuristik.
Hosten kompletterar modellkontexten med locale-namn och en explicit
`response_language_instruction`, upprepar instruktionen på det genererade
textfältets JSON-schema och placerar den sist i den strukturerade kontexten.
Det behövdes i liveprov för att valt språk skulle vinna även över en prompt
och konversationshistorik som uttryckligen begär motsatt språk.

Tekniska identiteter som `weather.current`, eventtyper, ID:n, hashes,
modellnamn och rå JSON översätts aldrig. Etiketter, statusförklaringar,
tillgänglighetstext, placeholders, datum och tider följer däremot vald locale.
Fler språk läggs till som kataloger och tas därefter upp i samma explicita
locale-allowlist; agent- och säkerhetslogiken behöver inte förgrenas per
språk.

## Mikrofon och STT

Talk-knappen är byggd på browserstandarderna `getUserMedia`, Web Audio och
`AudioWorklet`. Den använder inte `SpeechRecognition`, en Safari-specifik
implementation, user-agent-detektering eller någon molnbaserad fallback.

Flödet är:

```text
Talk → nivå/VAD → tystnadsstopp → PCM16-WAV → lokal STT
     → färskt transkript → samma versionskontrollerade agentformulär som text
```

Mikrofoninställningarna ligger separat och sparas endast i browserns lokala
lagring:

- fysisk mikrofon eller systemets standard,
- `auto`, svenska eller engelska som första explicita språkval,
- känslighet med levande nivåmätare och synlig signaltröskel,
- tystnadstid före automatiskt stopp och maximal taltid,
- echo cancellation, noise suppression och automatisk gain,
- om den valda mikrofonströmmen ska hållas varm mellan yttranden,
- automatiskt skicka eller lämna transkriptet redigerbart.

Känsligheten är en deterministisk signaltröskel, inte en semantisk
klassificerare och inte hårdvaruförstärkning. Upp till 1,5 sekunders pre-roll
behålls när tal upptäcks; längre väntetystnad trimmas och mycket korta segment
fylls till backendens minsta säkra WAV-längd. Det ger externa mikrofoners
signalbehandling och en låg första stavelse marginal utan att behålla
obegränsad väntetystnad.

Med **håll mikrofonen redo** aktiverat förblir browserns valda MediaStream och
AudioContext öppna, så browserns mikrofonindikator kan fortsätta vara synlig.
Själva PCM-workleten är däremot stoppad mellan yttranden: den buffrar eller
publicerar inget ljud förrän ett generationsmärkt `start` har skickats och
nollställer bufferten vid både `start` och `stop`. Om inställningen stängs av
frigörs ström, ljudgraf och MessagePort.

Råljud finns i applikationsminnet endast under en aktiv, begränsad inspelning,
medan ett bounded jobb väntar eller medan ett lokalt provideranrop kör. Det
persisteras inte och eventloggen får varken ljud, ljudhash eller transkript.
Transkript har en hostägd leverans-TTL; gamla resultat får inte auto-skickas.
Cancel rensar köat ljud direkt. Ett redan påbörjat provideranrop kan behålla
sin lokala request tills dess deadline, vilket redovisas som
`audio.retained`; dess sena resultat kastas.
Browsern skapar request-ID:t före uppladdningen och kan därför avbryta även
om POST-svaret ännu inte har kommit; en kortlivad, bounded tombstone gör att
en samtidigt anländande uppladdning nekas.

STT-workern är fristående från researchworkern och från robotens framtida
MotionSupervisor. En långsam eller felande transkribering får därför inte
blockera dialog, navigation eller den deterministiska säkerhetsloopen.

## Sex ytor

- **Arbetsbänk** visar en versionsmärkt konversation, pågående tur,
  verifierat slutsvar, typad aktivitet och eventuell evidens.
- **Karta** visar en skrivskyddad snapshot av robotens osäkra lokala
  världsminne. Simulatorn kan visa metriska `FREE`/`UNKNOWN`/`OCCUPIED`-rutor;
  EV3 visar i stället encoderbaserad lokal pose och provisoriska, icke-metriska
  IR-sektorer med samma hypotes-ID:n som den fysiska navigationen använder.
- **Kroppar** visar logiska robotar med controllers och perceptionskällor.
  BLAST visar den långlivade BLE-sessionen och kan anslutas, kopplas från
  eller återanslutas här. EV3 visar i stället resultatet från en explicit,
  rörelsefri beredskapskontroll; dess korta SSH-session stängs efter kontrollen
  och öppnas på nytt för varje uppdrag.
- **Händelser** visar den begränsade, append-only eventströmmen med
  korrelations-ID:n. Rå prompt, rå modelltext och fulla evidence-URL:er
  loggas inte.
- **Experiment** reserverar en read-only yta för reproducerbara episoder och
  befintliga experimentartefakter.
- **Inställningar** ändrar processbundna agentbudgetar och browserlokala
  mikrofonval. Agentinställningarna versionskontrolleras och återställs när
  servern startas om; mikrofoninställningarna lämnar aldrig browsern.

## Dialog och kontext

Varje tur har ett explicit läge:

- `conversation` tillåter ett direkt lokalt svar och låter fortfarande
  modellen välja ett godkänt informationsverktyg om färsk data behövs.
- `research_required` kräver verifierad evidens innan modellen får svara.

Hosten klassificerar inte användartext med regexp, substringmatchning eller
nyckelord. Gemma får en typad `conversation_history` med tidigare synliga
user/assistant-meddelanden och returnerar ett strikt `CALL_TOOL`, `ANSWER`,
`CLARIFY` eller `ABORT`. Det enda registrerade externa verktyget i denna
version är `weather.current`.

Dialoghistoriken är ett konversationsminne, inte robotens framtida
världsmodell. Fysiska följdreferenser som “två gånger till” måste senare
bindas till en separat, explicit action-/state-kontext innan någon exekvering
kan bli aktuell.

## Kartvyn

Dashboarden pollar den autentiserade, read-only routen `GET /api/v1/map`.
Resultatet är en frikopplad JSON-snapshot med fast storleksgräns; browsern
får varken kartans skrivcapability eller någon motorcapability.

Den aktuella simulatorn publicerar tre strålar per snapshot, framåt och
`±45°`. Avstånden beskriver robotradie-inflated configuration space och ska
inte tolkas som exakta fysiska objektytor. Kartan fusionerar upprepad positiv
och negativ evidens genom `UNKNOWN`, så en motstridig mätning vänder inte
omedelbart en ruta från upptagen till fri eller tvärtom. Sammanhängande
upptagna rutor blir persistenta, semantiskt opaka `UNKNOWN`-hypoteser. Det
ger en tydlig framtida söm där kamera-, ljud- eller LLM-klassificering kan
lägga till en etikett och evidens utan att ändra den geometriska kartan.

Fysisk EV3 `IR-PROX` är en reflektionssignal och får därför bara visas som
provisorisk kvalitativ evidens. Efter varje verifierad fysisk minnesuppdatering
publicerar runtime:n en frikopplad snapshot med lokal pose, färsk IR-relation
och de auktoritativa hinderhypoteserna. Ett separat SVG-lager ritar robotens
riktning och fasta screen-space-sektorer vid observationsposerna. Sektorlängden
är uttryckligen icke-metrisk: systemet hittar inte på millimeter, endpoint,
objektidentitet eller positivt fri väg. Om encoderlokaliseringen blir ogiltig
markeras kartan `degraded` och den osäkra posen ritas inte som aktuell.

Den fysiska projektionen visar nu även två nya, strikt faktabaserade lager:

- `collision_geometry` ritas som den konfigurerade asymmetriska EV3-kroppen
  med separata extents fram, bak, vänster och höger samt clearance-marginal.
  Provenance och kalibreringsstatus följer snapshoten. Konturen betyder inte
  att roboten kan känna sidokontakt och måtten är fortfarande provisoriska.
- `scan_evidence_history` ritar varje bevarad scans faktiska
  encoderhärledda kroppsvinklar från den pose där scannen utfördes. Blockerade
  och klara strålar skiljs visuellt, och panelen visar täckning,
  hypotesrelation och gränsstatus. Strålarna får ingen uppfunnen fysisk längd
  eller objektyta.

Scanprojektionen använder den episodlokala, diversity-bevarande historiken
från fysisk navigation memory, högst 16 försök per hinder och 64 i kartan.
Det auktoritativa minnet räknar dessutom kumulativt varje äldre scanpost som
har lämnat detaljhistoriken, med typad senaste orsak; räknaren överlever både
save/load och att en hel hinderhypotes lämnar kartan. Projektionen är alltså
inte en animation som browsern själv härleder. Samma
positiv-vänster/negativ-höger-konvention används av runtime, kontrakt och UI.
En äldre scan ritas vid sin historiska pose; den flyttas inte till robotens
nuvarande kropp.

Det strukturerade action/result-ledger som Gemma använder är däremot ännu
inte en egen dashboardpanel. Den är episodlokal plannerkontext och
runtime-telemetri, medan kartan visar dess fysiska underlag: pose, kropp,
hinder och scan-evidens. Detta undviker att dokumentationen lovar en
redigerbar eller fullständig beslutslogg i kartvyn.

Kartpublicering är best effort och har ingen motorauktoritet. Ett avvisat eller
felande kartanrop loggas som telemetri men kan varken rulla tillbaka fysisk
navigation memory eller stoppa nästa motorbeslut.

### Uppdrag, plan och tidslinje

Kartvyn visar även robotkontrollens aktuella uppdrag utan att blanda ihop
visning och exekvering. Den alltid synliga sammanfattningen innehåller mål,
kontrolltillstånd, aktuell semantisk handling, den ordnade modellplanen, den
typade lokala omvägsruttens status och aktiva waypoint, talstatus och
runtime:ns senaste lägeskommentar. Ruttpanelen visar modellens valda sida samt
avslutade, aktiva och återstående waypoints; den väljer aldrig en sida och har
ingen motorauktoritet. När episoden är avslutad märks panelen uttryckligen som
det senaste avslutade uppdraget så att en gammal plan inte ser aktiv ut.

Den utfällbara delen cursor-pollar två separata, autentiserade read-only-flöden:

- `GET /api/v1/robot/snapshots` för strukturerade ändringar av plan, handling,
  aktiv rutt/waypoint, hinderbedömning, scan, tal, kommentar och fel,
- `GET /api/v1/robot/events` för typade livscykel-, stopp- och felhändelser.

Event- och snapshotsekvenserna är oberoende. Browsern behåller därför en egen
cursor per flöde, deduplicerar på typ och sekvens och använder timestamp endast
för sammanvävning. En snapshot där bara modellatensen ändrats skapar ingen ny
användarhändelse. `robot.runtime_update` visas inte en andra gång när den rikare
snapshoten redan beskriver samma förändring. Hostens separata event- och
snapshotringar behåller vardera högst 4 096 poster. Browsern behåller de
senaste 1 000 från vartdera flödet och renderar högst 500 relevanta
tidslinjeposter. Ringarna har dessutom hårda bytebudgetar på 8 MiB för event
och 32 MiB för snapshots, och varje historiesida har en exakt gräns på 4 MiB.
Vid en backlog hämtar browsern högst fyra sidor per omgång och fortsätter
omedelbart i nästa omgång; statusen visar `hämtar ikapp` tills båda cursorerna
nått respektive senaste sekvens. En synlig gapmarkör talar om när backend eller
browser har
släppt äldre poster, eller när den längre lokala tidslinjen har kapats;
frekventa duplicerade runtime-event tar inte plats från dessa poster. All
denna historik är processlokal observabilitet, inte robotens beständiga
navigationsminne eller rå plannerkontext.

Ett modellförslag som nekas av det deterministiska kontraktet publicerar sin
typade vetoorsak i lägeskommentaren. Det gör exempelvis ett för tidigt
`FINISH` synligt utan att rå modelltext eller prompt behöver loggas.

Kartan ritar dessutom `pose_history`, högst 2 048 lokala odometriposer i ordning.
Oförändrad position och riktning dedupliceras exakt, medan rotation på plats
behålls. Äldre punkter räknas när taket nås och historiken nollställs vid ny
världsmodell eller koordinatram. Spåret är en uppskattning från encoderodometri,
inte fysisk ground truth, och det överlever ännu inte en processomstart.

Den fysiska dashboardprojektionen behåller också högst 1 024 kvalitativa
IR-observationer och 64 posebundna scanposter. Endast de 100 senaste
kvalitativa observationerna materialiseras som DOM-kort. Panelen visar antal
renderade, bevarade och borttagna poster; även scanposter som lämnar den heta
processprojektionen räknas. Separat kartmetadata visar det auktoritativa
hazard- och scanminnets bevarade antal, kapacitet, kumulativa eviction-antal
och senaste typade scanorsak. Hela
det slutliga HTTP-svaret från kart-endpointen har en
hård gräns på 4 MiB. Dessa generösa men ändliga tak gör att ett längre experiment
kan inspekteras utan att gamla fakta försvinner efter bara ett fåtal steg,
samtidigt som pose-eviction och tidslinjegap fortfarande är räknade eller
synliga.

Detta är ännu inte SLAM, global lokalisering, A*, frontier exploration eller
ett navigationsfacit. Fysisk navigation använder sitt auktoritativa hazard
memory; dashboardprojektionen är endast en read-only vy av samma fakta och kan
inte skriva tillbaka. Framtida kameror och flera LEGO-controllers måste
publicera explicita koordinatramar och kalibrerade transformeringar;
observationer från olika ramar får inte slås ihop bara för att de tillhör samma
logiska robot.

## Kontroll- och säkerhetsgräns

Dashboarden kan starta och stoppa ett mål endast när processen uttryckligen
har fått en fysisk runtime-adapter, exempelvis via EV3-startprofilen. HTTP-lagret
skriver aldrig direkt till motorer, SSH eller TTS: det lämnar ett typat mål
eller stoppkrav till robotkontrolltjänsten, som i sin tur använder den enda
serialiserade physical runtime som får äga motortransporten. Utan en injicerad
adapter är kontrollplanet `DISABLED` och muterande robotanrop nekas.

Anslutningskontrollerna följer samma gräns. BLAST-knapparna styr den
konfigurerade BLE-livscykeln. EV3-knappen kör en avgränsad preflight utan
motoranrop och delar ett exklusivt controller-lås med fysiska uppdrag, så en
kontroll och en navigationsepisod aldrig kan använda bricken samtidigt.

Kartan, komponentregistret och tekniska historikvyer är read-only projektioner.
Att ett nodkort visar `online` ger därför inte nodkortet egen motorauktoritet,
och kartan kan aldrig skriva tillbaka ett rörelsebeslut.

Webbgränsen använder dessutom:

- en slumpad 256-bitars åtkomstnyckel i en owner-only-fil för den normala
  startprofilen
- en tokeniserad `/live/<access-key>/`-sökväg och samma nyckelkrav på samtliga API-anrop,
  även läsningar
- exakt `Host`- och `Origin`-kontroll
- strikt JSON utan dubblettnycklar eller icke-finita tal
- fasta request- och responsegränser
- en explicit asset- och route-allowlist
- en strikt Content Security Policy utan externa assets
- socket-timeout och ett fast tak för samtidiga HTTP-handlers
- separata begränsade jobbköer för research och STT

Frontend anropar endast hostservern. Den kontaktar aldrig LM Studio,
Open-Meteo eller EV3 direkt.

## Komponentmodell

Registret skiljer på en logisk robot och dess noder:

```text
EV3RSTORM
├── EV3 Main · controller
├── Front Camera · framtida camera source
└── Microphone Array · framtida microphone source

Composite Lab Robot
├── Robot Inventor 51515 · framtida controller
├── BOOST Move Hub · framtida controller
├── Vision Node · framtida camera source
└── Audio Node · framtida microphone source
```

Alla poster är beskrivande och har `control_exposed: false`. Samma kontrakt
kan senare representera flera parallella controllers och perceptionskällor
utan att göra dashboarden till motorägare.
