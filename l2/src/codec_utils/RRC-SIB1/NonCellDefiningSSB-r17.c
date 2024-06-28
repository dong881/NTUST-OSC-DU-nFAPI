/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "NonCellDefiningSSB-r17.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_ssb_Periodicity_r17_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_ssb_TimeOffset_r17_constr_12 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_ssb_Periodicity_r17_value2enum_3[] = {
	{ 0,	3,	"ms5" },
	{ 1,	4,	"ms10" },
	{ 2,	4,	"ms20" },
	{ 3,	4,	"ms40" },
	{ 4,	4,	"ms80" },
	{ 5,	5,	"ms160" },
	{ 6,	6,	"spare2" },
	{ 7,	6,	"spare1" }
};
static const unsigned int asn_MAP_ssb_Periodicity_r17_enum2value_3[] = {
	1,	/* ms10(1) */
	5,	/* ms160(5) */
	2,	/* ms20(2) */
	3,	/* ms40(3) */
	0,	/* ms5(0) */
	4,	/* ms80(4) */
	7,	/* spare1(7) */
	6	/* spare2(6) */
};
static const asn_INTEGER_specifics_t asn_SPC_ssb_Periodicity_r17_specs_3 = {
	asn_MAP_ssb_Periodicity_r17_value2enum_3,	/* "tag" => N; sorted by tag */
	asn_MAP_ssb_Periodicity_r17_enum2value_3,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_ssb_Periodicity_r17_tags_3[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_ssb_Periodicity_r17_3 = {
	"ssb-Periodicity-r17",
	"ssb-Periodicity-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_ssb_Periodicity_r17_tags_3,
	sizeof(asn_DEF_ssb_Periodicity_r17_tags_3)
		/sizeof(asn_DEF_ssb_Periodicity_r17_tags_3[0]) - 1, /* 1 */
	asn_DEF_ssb_Periodicity_r17_tags_3,	/* Same as above */
	sizeof(asn_DEF_ssb_Periodicity_r17_tags_3)
		/sizeof(asn_DEF_ssb_Periodicity_r17_tags_3[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_ssb_Periodicity_r17_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_ssb_Periodicity_r17_specs_3	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_ssb_TimeOffset_r17_value2enum_12[] = {
	{ 0,	3,	"ms5" },
	{ 1,	4,	"ms10" },
	{ 2,	4,	"ms15" },
	{ 3,	4,	"ms20" },
	{ 4,	4,	"ms40" },
	{ 5,	4,	"ms80" },
	{ 6,	6,	"spare2" },
	{ 7,	6,	"spare1" }
};
static const unsigned int asn_MAP_ssb_TimeOffset_r17_enum2value_12[] = {
	1,	/* ms10(1) */
	2,	/* ms15(2) */
	3,	/* ms20(3) */
	4,	/* ms40(4) */
	0,	/* ms5(0) */
	5,	/* ms80(5) */
	7,	/* spare1(7) */
	6	/* spare2(6) */
};
static const asn_INTEGER_specifics_t asn_SPC_ssb_TimeOffset_r17_specs_12 = {
	asn_MAP_ssb_TimeOffset_r17_value2enum_12,	/* "tag" => N; sorted by tag */
	asn_MAP_ssb_TimeOffset_r17_enum2value_12,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_ssb_TimeOffset_r17_tags_12[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_ssb_TimeOffset_r17_12 = {
	"ssb-TimeOffset-r17",
	"ssb-TimeOffset-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_ssb_TimeOffset_r17_tags_12,
	sizeof(asn_DEF_ssb_TimeOffset_r17_tags_12)
		/sizeof(asn_DEF_ssb_TimeOffset_r17_tags_12[0]) - 1, /* 1 */
	asn_DEF_ssb_TimeOffset_r17_tags_12,	/* Same as above */
	sizeof(asn_DEF_ssb_TimeOffset_r17_tags_12)
		/sizeof(asn_DEF_ssb_TimeOffset_r17_tags_12[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_ssb_TimeOffset_r17_constr_12,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_ssb_TimeOffset_r17_specs_12	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_NonCellDefiningSSB_r17_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct NonCellDefiningSSB_r17, absoluteFrequencySSB_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ARFCN_ValueNR,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"absoluteFrequencySSB-r17"
		},
	{ ATF_POINTER, 2, offsetof(struct NonCellDefiningSSB_r17, ssb_Periodicity_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ssb_Periodicity_r17_3,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"ssb-Periodicity-r17"
		},
	{ ATF_POINTER, 1, offsetof(struct NonCellDefiningSSB_r17, ssb_TimeOffset_r17),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ssb_TimeOffset_r17_12,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"ssb-TimeOffset-r17"
		},
};
static const int asn_MAP_NonCellDefiningSSB_r17_oms_1[] = { 1, 2 };
static const ber_tlv_tag_t asn_DEF_NonCellDefiningSSB_r17_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_NonCellDefiningSSB_r17_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* absoluteFrequencySSB-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* ssb-Periodicity-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* ssb-TimeOffset-r17 */
};
asn_SEQUENCE_specifics_t asn_SPC_NonCellDefiningSSB_r17_specs_1 = {
	sizeof(struct NonCellDefiningSSB_r17),
	offsetof(struct NonCellDefiningSSB_r17, _asn_ctx),
	asn_MAP_NonCellDefiningSSB_r17_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_NonCellDefiningSSB_r17_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_NonCellDefiningSSB_r17 = {
	"NonCellDefiningSSB-r17",
	"NonCellDefiningSSB-r17",
	&asn_OP_SEQUENCE,
	asn_DEF_NonCellDefiningSSB_r17_tags_1,
	sizeof(asn_DEF_NonCellDefiningSSB_r17_tags_1)
		/sizeof(asn_DEF_NonCellDefiningSSB_r17_tags_1[0]), /* 1 */
	asn_DEF_NonCellDefiningSSB_r17_tags_1,	/* Same as above */
	sizeof(asn_DEF_NonCellDefiningSSB_r17_tags_1)
		/sizeof(asn_DEF_NonCellDefiningSSB_r17_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_NonCellDefiningSSB_r17_1,
	3,	/* Elements count */
	&asn_SPC_NonCellDefiningSSB_r17_specs_1	/* Additional specs */
};

