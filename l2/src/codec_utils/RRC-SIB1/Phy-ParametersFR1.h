/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_Phy_ParametersFR1_H_
#define	_Phy_ParametersFR1_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum Phy_ParametersFR1__pdcch_MonitoringSingleOccasion {
	Phy_ParametersFR1__pdcch_MonitoringSingleOccasion_supported	= 0
} e_Phy_ParametersFR1__pdcch_MonitoringSingleOccasion;
typedef enum Phy_ParametersFR1__scs_60kHz {
	Phy_ParametersFR1__scs_60kHz_supported	= 0
} e_Phy_ParametersFR1__scs_60kHz;
typedef enum Phy_ParametersFR1__pdsch_256QAM_FR1 {
	Phy_ParametersFR1__pdsch_256QAM_FR1_supported	= 0
} e_Phy_ParametersFR1__pdsch_256QAM_FR1;
typedef enum Phy_ParametersFR1__pdsch_RE_MappingFR1_PerSymbol {
	Phy_ParametersFR1__pdsch_RE_MappingFR1_PerSymbol_n10	= 0,
	Phy_ParametersFR1__pdsch_RE_MappingFR1_PerSymbol_n20	= 1
} e_Phy_ParametersFR1__pdsch_RE_MappingFR1_PerSymbol;
typedef enum Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot {
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n16	= 0,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n32	= 1,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n48	= 2,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n64	= 3,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n80	= 4,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n96	= 5,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n112	= 6,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n128	= 7,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n144	= 8,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n160	= 9,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n176	= 10,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n192	= 11,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n208	= 12,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n224	= 13,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n240	= 14,
	Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot_n256	= 15
} e_Phy_ParametersFR1__ext1__pdsch_RE_MappingFR1_PerSlot;
typedef enum Phy_ParametersFR1__ext2__pdcch_MonitoringSingleSpanFirst4Sym_r16 {
	Phy_ParametersFR1__ext2__pdcch_MonitoringSingleSpanFirst4Sym_r16_supported	= 0
} e_Phy_ParametersFR1__ext2__pdcch_MonitoringSingleSpanFirst4Sym_r16;

/* Phy-ParametersFR1 */
typedef struct Phy_ParametersFR1 {
	long	*pdcch_MonitoringSingleOccasion;	/* OPTIONAL */
	long	*scs_60kHz;	/* OPTIONAL */
	long	*pdsch_256QAM_FR1;	/* OPTIONAL */
	long	*pdsch_RE_MappingFR1_PerSymbol;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct Phy_ParametersFR1__ext1 {
		long	*pdsch_RE_MappingFR1_PerSlot;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct Phy_ParametersFR1__ext2 {
		long	*pdcch_MonitoringSingleSpanFirst4Sym_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Phy_ParametersFR1_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_pdcch_MonitoringSingleOccasion_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_scs_60kHz_4;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_pdsch_256QAM_FR1_6;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_pdsch_RE_MappingFR1_PerSymbol_8;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_pdsch_RE_MappingFR1_PerSlot_13;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_pdcch_MonitoringSingleSpanFirst4Sym_r16_31;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_Phy_ParametersFR1;
extern asn_SEQUENCE_specifics_t asn_SPC_Phy_ParametersFR1_specs_1;
extern asn_TYPE_member_t asn_MBR_Phy_ParametersFR1_1[6];

#ifdef __cplusplus
}
#endif

#endif	/* _Phy_ParametersFR1_H_ */
#include <asn_internal.h>
