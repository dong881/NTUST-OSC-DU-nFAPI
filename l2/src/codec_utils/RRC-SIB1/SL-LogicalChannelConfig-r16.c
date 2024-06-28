/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "SL-LogicalChannelConfig-r16.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_sl_Priority_r16_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 1L && value <= 8L)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_sl_AllowedCG_List_r16_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	/* Determine the number of elements */
	size = _A_CSEQUENCE_FROM_VOID(sptr)->count;
	
	if((size <= 7UL)) {
		/* Perform validation of the inner elements */
		return SEQUENCE_OF_constraint(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_sl_AllowedSCS_List_r16_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	/* Determine the number of elements */
	size = _A_CSEQUENCE_FROM_VOID(sptr)->count;
	
	if((size >= 1UL && size <= 5UL)) {
		/* Perform validation of the inner elements */
		return SEQUENCE_OF_constraint(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_sl_LogicalChannelGroup_r16_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0L && value <= 7L)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_sl_PrioritisedBitRate_r16_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  15 }	/* (0..15) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_sl_BucketSizeDuration_r16_constr_20 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  15 }	/* (0..15) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_sl_ConfiguredGrantType1Allowed_r16_constr_37 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_sl_HARQ_FeedbackEnabled_r16_constr_39 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 1,  1,  0,  1 }	/* (0..1) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_sl_AllowedCG_List_r16_constr_42 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (SIZE(0..7)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_sl_AllowedSCS_List_r16_constr_44 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 3,  3,  1,  5 }	/* (SIZE(1..5)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_sl_MaxPUSCH_Duration_r16_constr_46 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_sl_Priority_r16_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  1,  8 }	/* (1..8) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_sl_AllowedCG_List_r16_constr_42 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (SIZE(0..7)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_sl_AllowedSCS_List_r16_constr_44 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 3,  3,  1,  5 }	/* (SIZE(1..5)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_sl_LogicalChannelGroup_r16_constr_55 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_sl_PrioritisedBitRate_r16_value2enum_3[] = {
	{ 0,	5,	"kBps0" },
	{ 1,	5,	"kBps8" },
	{ 2,	6,	"kBps16" },
	{ 3,	6,	"kBps32" },
	{ 4,	6,	"kBps64" },
	{ 5,	7,	"kBps128" },
	{ 6,	7,	"kBps256" },
	{ 7,	7,	"kBps512" },
	{ 8,	8,	"kBps1024" },
	{ 9,	8,	"kBps2048" },
	{ 10,	8,	"kBps4096" },
	{ 11,	8,	"kBps8192" },
	{ 12,	9,	"kBps16384" },
	{ 13,	9,	"kBps32768" },
	{ 14,	9,	"kBps65536" },
	{ 15,	8,	"infinity" }
};
static const unsigned int asn_MAP_sl_PrioritisedBitRate_r16_enum2value_3[] = {
	15,	/* infinity(15) */
	0,	/* kBps0(0) */
	8,	/* kBps1024(8) */
	5,	/* kBps128(5) */
	2,	/* kBps16(2) */
	12,	/* kBps16384(12) */
	9,	/* kBps2048(9) */
	6,	/* kBps256(6) */
	3,	/* kBps32(3) */
	13,	/* kBps32768(13) */
	10,	/* kBps4096(10) */
	7,	/* kBps512(7) */
	4,	/* kBps64(4) */
	14,	/* kBps65536(14) */
	1,	/* kBps8(1) */
	11	/* kBps8192(11) */
};
static const asn_INTEGER_specifics_t asn_SPC_sl_PrioritisedBitRate_r16_specs_3 = {
	asn_MAP_sl_PrioritisedBitRate_r16_value2enum_3,	/* "tag" => N; sorted by tag */
	asn_MAP_sl_PrioritisedBitRate_r16_enum2value_3,	/* N => "tag"; sorted by N */
	16,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_sl_PrioritisedBitRate_r16_tags_3[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_sl_PrioritisedBitRate_r16_3 = {
	"sl-PrioritisedBitRate-r16",
	"sl-PrioritisedBitRate-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_sl_PrioritisedBitRate_r16_tags_3,
	sizeof(asn_DEF_sl_PrioritisedBitRate_r16_tags_3)
		/sizeof(asn_DEF_sl_PrioritisedBitRate_r16_tags_3[0]) - 1, /* 1 */
	asn_DEF_sl_PrioritisedBitRate_r16_tags_3,	/* Same as above */
	sizeof(asn_DEF_sl_PrioritisedBitRate_r16_tags_3)
		/sizeof(asn_DEF_sl_PrioritisedBitRate_r16_tags_3[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_sl_PrioritisedBitRate_r16_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_sl_PrioritisedBitRate_r16_specs_3	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_sl_BucketSizeDuration_r16_value2enum_20[] = {
	{ 0,	3,	"ms5" },
	{ 1,	4,	"ms10" },
	{ 2,	4,	"ms20" },
	{ 3,	4,	"ms50" },
	{ 4,	5,	"ms100" },
	{ 5,	5,	"ms150" },
	{ 6,	5,	"ms300" },
	{ 7,	5,	"ms500" },
	{ 8,	6,	"ms1000" },
	{ 9,	6,	"spare7" },
	{ 10,	6,	"spare6" },
	{ 11,	6,	"spare5" },
	{ 12,	6,	"spare4" },
	{ 13,	6,	"spare3" },
	{ 14,	6,	"spare2" },
	{ 15,	6,	"spare1" }
};
static const unsigned int asn_MAP_sl_BucketSizeDuration_r16_enum2value_20[] = {
	1,	/* ms10(1) */
	4,	/* ms100(4) */
	8,	/* ms1000(8) */
	5,	/* ms150(5) */
	2,	/* ms20(2) */
	6,	/* ms300(6) */
	0,	/* ms5(0) */
	3,	/* ms50(3) */
	7,	/* ms500(7) */
	15,	/* spare1(15) */
	14,	/* spare2(14) */
	13,	/* spare3(13) */
	12,	/* spare4(12) */
	11,	/* spare5(11) */
	10,	/* spare6(10) */
	9	/* spare7(9) */
};
static const asn_INTEGER_specifics_t asn_SPC_sl_BucketSizeDuration_r16_specs_20 = {
	asn_MAP_sl_BucketSizeDuration_r16_value2enum_20,	/* "tag" => N; sorted by tag */
	asn_MAP_sl_BucketSizeDuration_r16_enum2value_20,	/* N => "tag"; sorted by N */
	16,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_sl_BucketSizeDuration_r16_tags_20[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_sl_BucketSizeDuration_r16_20 = {
	"sl-BucketSizeDuration-r16",
	"sl-BucketSizeDuration-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_sl_BucketSizeDuration_r16_tags_20,
	sizeof(asn_DEF_sl_BucketSizeDuration_r16_tags_20)
		/sizeof(asn_DEF_sl_BucketSizeDuration_r16_tags_20[0]) - 1, /* 1 */
	asn_DEF_sl_BucketSizeDuration_r16_tags_20,	/* Same as above */
	sizeof(asn_DEF_sl_BucketSizeDuration_r16_tags_20)
		/sizeof(asn_DEF_sl_BucketSizeDuration_r16_tags_20[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_sl_BucketSizeDuration_r16_constr_20,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_sl_BucketSizeDuration_r16_specs_20	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_sl_ConfiguredGrantType1Allowed_r16_value2enum_37[] = {
	{ 0,	4,	"true" }
};
static const unsigned int asn_MAP_sl_ConfiguredGrantType1Allowed_r16_enum2value_37[] = {
	0	/* true(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_sl_ConfiguredGrantType1Allowed_r16_specs_37 = {
	asn_MAP_sl_ConfiguredGrantType1Allowed_r16_value2enum_37,	/* "tag" => N; sorted by tag */
	asn_MAP_sl_ConfiguredGrantType1Allowed_r16_enum2value_37,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_sl_ConfiguredGrantType1Allowed_r16_tags_37[] = {
	(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_sl_ConfiguredGrantType1Allowed_r16_37 = {
	"sl-ConfiguredGrantType1Allowed-r16",
	"sl-ConfiguredGrantType1Allowed-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_sl_ConfiguredGrantType1Allowed_r16_tags_37,
	sizeof(asn_DEF_sl_ConfiguredGrantType1Allowed_r16_tags_37)
		/sizeof(asn_DEF_sl_ConfiguredGrantType1Allowed_r16_tags_37[0]) - 1, /* 1 */
	asn_DEF_sl_ConfiguredGrantType1Allowed_r16_tags_37,	/* Same as above */
	sizeof(asn_DEF_sl_ConfiguredGrantType1Allowed_r16_tags_37)
		/sizeof(asn_DEF_sl_ConfiguredGrantType1Allowed_r16_tags_37[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_sl_ConfiguredGrantType1Allowed_r16_constr_37,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_sl_ConfiguredGrantType1Allowed_r16_specs_37	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_sl_HARQ_FeedbackEnabled_r16_value2enum_39[] = {
	{ 0,	7,	"enabled" },
	{ 1,	8,	"disabled" }
};
static const unsigned int asn_MAP_sl_HARQ_FeedbackEnabled_r16_enum2value_39[] = {
	1,	/* disabled(1) */
	0	/* enabled(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_sl_HARQ_FeedbackEnabled_r16_specs_39 = {
	asn_MAP_sl_HARQ_FeedbackEnabled_r16_value2enum_39,	/* "tag" => N; sorted by tag */
	asn_MAP_sl_HARQ_FeedbackEnabled_r16_enum2value_39,	/* N => "tag"; sorted by N */
	2,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_sl_HARQ_FeedbackEnabled_r16_tags_39[] = {
	(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_sl_HARQ_FeedbackEnabled_r16_39 = {
	"sl-HARQ-FeedbackEnabled-r16",
	"sl-HARQ-FeedbackEnabled-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_sl_HARQ_FeedbackEnabled_r16_tags_39,
	sizeof(asn_DEF_sl_HARQ_FeedbackEnabled_r16_tags_39)
		/sizeof(asn_DEF_sl_HARQ_FeedbackEnabled_r16_tags_39[0]) - 1, /* 1 */
	asn_DEF_sl_HARQ_FeedbackEnabled_r16_tags_39,	/* Same as above */
	sizeof(asn_DEF_sl_HARQ_FeedbackEnabled_r16_tags_39)
		/sizeof(asn_DEF_sl_HARQ_FeedbackEnabled_r16_tags_39[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_sl_HARQ_FeedbackEnabled_r16_constr_39,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_sl_HARQ_FeedbackEnabled_r16_specs_39	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_sl_AllowedCG_List_r16_42[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (2 << 2)),
		0,
		&asn_DEF_SL_ConfigIndexCG_r16,
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
		""
		},
};
static const ber_tlv_tag_t asn_DEF_sl_AllowedCG_List_r16_tags_42[] = {
	(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_sl_AllowedCG_List_r16_specs_42 = {
	sizeof(struct SL_LogicalChannelConfig_r16__sl_AllowedCG_List_r16),
	offsetof(struct SL_LogicalChannelConfig_r16__sl_AllowedCG_List_r16, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_sl_AllowedCG_List_r16_42 = {
	"sl-AllowedCG-List-r16",
	"sl-AllowedCG-List-r16",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_sl_AllowedCG_List_r16_tags_42,
	sizeof(asn_DEF_sl_AllowedCG_List_r16_tags_42)
		/sizeof(asn_DEF_sl_AllowedCG_List_r16_tags_42[0]) - 1, /* 1 */
	asn_DEF_sl_AllowedCG_List_r16_tags_42,	/* Same as above */
	sizeof(asn_DEF_sl_AllowedCG_List_r16_tags_42)
		/sizeof(asn_DEF_sl_AllowedCG_List_r16_tags_42[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_sl_AllowedCG_List_r16_constr_42,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_OF_constraint
	},
	asn_MBR_sl_AllowedCG_List_r16_42,
	1,	/* Single element */
	&asn_SPC_sl_AllowedCG_List_r16_specs_42	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_sl_AllowedSCS_List_r16_44[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (10 << 2)),
		0,
		&asn_DEF_SubcarrierSpacing,
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
		""
		},
};
static const ber_tlv_tag_t asn_DEF_sl_AllowedSCS_List_r16_tags_44[] = {
	(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_sl_AllowedSCS_List_r16_specs_44 = {
	sizeof(struct SL_LogicalChannelConfig_r16__sl_AllowedSCS_List_r16),
	offsetof(struct SL_LogicalChannelConfig_r16__sl_AllowedSCS_List_r16, _asn_ctx),
	1,	/* XER encoding is XMLValueList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_sl_AllowedSCS_List_r16_44 = {
	"sl-AllowedSCS-List-r16",
	"sl-AllowedSCS-List-r16",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_sl_AllowedSCS_List_r16_tags_44,
	sizeof(asn_DEF_sl_AllowedSCS_List_r16_tags_44)
		/sizeof(asn_DEF_sl_AllowedSCS_List_r16_tags_44[0]) - 1, /* 1 */
	asn_DEF_sl_AllowedSCS_List_r16_tags_44,	/* Same as above */
	sizeof(asn_DEF_sl_AllowedSCS_List_r16_tags_44)
		/sizeof(asn_DEF_sl_AllowedSCS_List_r16_tags_44[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_sl_AllowedSCS_List_r16_constr_44,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_OF_constraint
	},
	asn_MBR_sl_AllowedSCS_List_r16_44,
	1,	/* Single element */
	&asn_SPC_sl_AllowedSCS_List_r16_specs_44	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_sl_MaxPUSCH_Duration_r16_value2enum_46[] = {
	{ 0,	6,	"ms0p02" },
	{ 1,	6,	"ms0p04" },
	{ 2,	8,	"ms0p0625" },
	{ 3,	7,	"ms0p125" },
	{ 4,	6,	"ms0p25" },
	{ 5,	5,	"ms0p5" },
	{ 6,	6,	"spare2" },
	{ 7,	6,	"spare1" }
};
static const unsigned int asn_MAP_sl_MaxPUSCH_Duration_r16_enum2value_46[] = {
	0,	/* ms0p02(0) */
	1,	/* ms0p04(1) */
	2,	/* ms0p0625(2) */
	3,	/* ms0p125(3) */
	4,	/* ms0p25(4) */
	5,	/* ms0p5(5) */
	7,	/* spare1(7) */
	6	/* spare2(6) */
};
static const asn_INTEGER_specifics_t asn_SPC_sl_MaxPUSCH_Duration_r16_specs_46 = {
	asn_MAP_sl_MaxPUSCH_Duration_r16_value2enum_46,	/* "tag" => N; sorted by tag */
	asn_MAP_sl_MaxPUSCH_Duration_r16_enum2value_46,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_sl_MaxPUSCH_Duration_r16_tags_46[] = {
	(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_sl_MaxPUSCH_Duration_r16_46 = {
	"sl-MaxPUSCH-Duration-r16",
	"sl-MaxPUSCH-Duration-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_sl_MaxPUSCH_Duration_r16_tags_46,
	sizeof(asn_DEF_sl_MaxPUSCH_Duration_r16_tags_46)
		/sizeof(asn_DEF_sl_MaxPUSCH_Duration_r16_tags_46[0]) - 1, /* 1 */
	asn_DEF_sl_MaxPUSCH_Duration_r16_tags_46,	/* Same as above */
	sizeof(asn_DEF_sl_MaxPUSCH_Duration_r16_tags_46)
		/sizeof(asn_DEF_sl_MaxPUSCH_Duration_r16_tags_46[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_sl_MaxPUSCH_Duration_r16_constr_46,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_sl_MaxPUSCH_Duration_r16_specs_46	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_SL_LogicalChannelConfig_r16_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct SL_LogicalChannelConfig_r16, sl_Priority_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_sl_Priority_r16_constr_2,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_sl_Priority_r16_constraint_1
		},
		0, 0, /* No default value */
		"sl-Priority-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SL_LogicalChannelConfig_r16, sl_PrioritisedBitRate_r16),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_sl_PrioritisedBitRate_r16_3,
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
		"sl-PrioritisedBitRate-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SL_LogicalChannelConfig_r16, sl_BucketSizeDuration_r16),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_sl_BucketSizeDuration_r16_20,
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
		"sl-BucketSizeDuration-r16"
		},
	{ ATF_POINTER, 8, offsetof(struct SL_LogicalChannelConfig_r16, sl_ConfiguredGrantType1Allowed_r16),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_sl_ConfiguredGrantType1Allowed_r16_37,
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
		"sl-ConfiguredGrantType1Allowed-r16"
		},
	{ ATF_POINTER, 7, offsetof(struct SL_LogicalChannelConfig_r16, sl_HARQ_FeedbackEnabled_r16),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_sl_HARQ_FeedbackEnabled_r16_39,
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
		"sl-HARQ-FeedbackEnabled-r16"
		},
	{ ATF_POINTER, 6, offsetof(struct SL_LogicalChannelConfig_r16, sl_AllowedCG_List_r16),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		0,
		&asn_DEF_sl_AllowedCG_List_r16_42,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_sl_AllowedCG_List_r16_constr_42,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_sl_AllowedCG_List_r16_constraint_1
		},
		0, 0, /* No default value */
		"sl-AllowedCG-List-r16"
		},
	{ ATF_POINTER, 5, offsetof(struct SL_LogicalChannelConfig_r16, sl_AllowedSCS_List_r16),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		0,
		&asn_DEF_sl_AllowedSCS_List_r16_44,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_sl_AllowedSCS_List_r16_constr_44,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_sl_AllowedSCS_List_r16_constraint_1
		},
		0, 0, /* No default value */
		"sl-AllowedSCS-List-r16"
		},
	{ ATF_POINTER, 4, offsetof(struct SL_LogicalChannelConfig_r16, sl_MaxPUSCH_Duration_r16),
		(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_sl_MaxPUSCH_Duration_r16_46,
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
		"sl-MaxPUSCH-Duration-r16"
		},
	{ ATF_POINTER, 3, offsetof(struct SL_LogicalChannelConfig_r16, sl_LogicalChannelGroup_r16),
		(ASN_TAG_CLASS_CONTEXT | (8 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_sl_LogicalChannelGroup_r16_constr_55,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_sl_LogicalChannelGroup_r16_constraint_1
		},
		0, 0, /* No default value */
		"sl-LogicalChannelGroup-r16"
		},
	{ ATF_POINTER, 2, offsetof(struct SL_LogicalChannelConfig_r16, sl_SchedulingRequestId_r16),
		(ASN_TAG_CLASS_CONTEXT | (9 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SchedulingRequestId,
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
		"sl-SchedulingRequestId-r16"
		},
	{ ATF_POINTER, 1, offsetof(struct SL_LogicalChannelConfig_r16, sl_LogicalChannelSR_DelayTimerApplied_r16),
		(ASN_TAG_CLASS_CONTEXT | (10 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BOOLEAN,
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
		"sl-LogicalChannelSR-DelayTimerApplied-r16"
		},
};
static const int asn_MAP_SL_LogicalChannelConfig_r16_oms_1[] = { 3, 4, 5, 6, 7, 8, 9, 10 };
static const ber_tlv_tag_t asn_DEF_SL_LogicalChannelConfig_r16_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_SL_LogicalChannelConfig_r16_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* sl-Priority-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* sl-PrioritisedBitRate-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* sl-BucketSizeDuration-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* sl-ConfiguredGrantType1Allowed-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* sl-HARQ-FeedbackEnabled-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* sl-AllowedCG-List-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 }, /* sl-AllowedSCS-List-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (7 << 2)), 7, 0, 0 }, /* sl-MaxPUSCH-Duration-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (8 << 2)), 8, 0, 0 }, /* sl-LogicalChannelGroup-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (9 << 2)), 9, 0, 0 }, /* sl-SchedulingRequestId-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (10 << 2)), 10, 0, 0 } /* sl-LogicalChannelSR-DelayTimerApplied-r16 */
};
asn_SEQUENCE_specifics_t asn_SPC_SL_LogicalChannelConfig_r16_specs_1 = {
	sizeof(struct SL_LogicalChannelConfig_r16),
	offsetof(struct SL_LogicalChannelConfig_r16, _asn_ctx),
	asn_MAP_SL_LogicalChannelConfig_r16_tag2el_1,
	11,	/* Count of tags in the map */
	asn_MAP_SL_LogicalChannelConfig_r16_oms_1,	/* Optional members */
	8, 0,	/* Root/Additions */
	11,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_SL_LogicalChannelConfig_r16 = {
	"SL-LogicalChannelConfig-r16",
	"SL-LogicalChannelConfig-r16",
	&asn_OP_SEQUENCE,
	asn_DEF_SL_LogicalChannelConfig_r16_tags_1,
	sizeof(asn_DEF_SL_LogicalChannelConfig_r16_tags_1)
		/sizeof(asn_DEF_SL_LogicalChannelConfig_r16_tags_1[0]), /* 1 */
	asn_DEF_SL_LogicalChannelConfig_r16_tags_1,	/* Same as above */
	sizeof(asn_DEF_SL_LogicalChannelConfig_r16_tags_1)
		/sizeof(asn_DEF_SL_LogicalChannelConfig_r16_tags_1[0]), /* 1 */
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
	asn_MBR_SL_LogicalChannelConfig_r16_1,
	11,	/* Elements count */
	&asn_SPC_SL_LogicalChannelConfig_r16_specs_1	/* Additional specs */
};

